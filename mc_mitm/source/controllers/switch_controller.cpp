/*
 * Copyright (c) 2020-2023 ndeadly
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "switch_controller.hpp"
#include "../mcmitm_config.hpp"
#include <string>

#define ARRAY_SIZE(array) \
	((size_t) ((sizeof(array) / sizeof((array)[0]))))

static u32 hid_field_extract(u8 *report, unsigned offset, int n)
{
	unsigned int idx = offset / 8;
	unsigned int bit_nr = 0;
	unsigned int bit_shift = offset % 8;
	int bits_to_copy = 8 - bit_shift;
	u32 value = 0;
	u32 mask = n < 32 ? (1U << n) - 1 : ~0U;

	while (n > 0) {
		value |= ((u32)report[idx] >> bit_shift) << bit_nr;
		n -= bits_to_copy;
		bit_nr += bits_to_copy;
		bits_to_copy = 8;
		bit_shift = 0;
		idx++;
	}

	return value & mask;
}

#define JC_MAX_STICK_MAG		 32767
#define JC_CAL_USR_MAGIC_0		 0xB2
#define JC_CAL_USR_MAGIC_1		 0xA1

static bool has_cal_magic(u8 *reply)
{
	return reply[0] != JC_CAL_USR_MAGIC_0 || reply[1] != JC_CAL_USR_MAGIC_1;
}

namespace ams::controller {

    namespace {
        constexpr float stick_ratio = JOYSTICK_MAX / (STICK_MAX / 2.0);

        static s32 joycon_map_stick_val(struct joycon_stick_cal *cal, s32 val)
        {
            s32 center = cal->center;
            s32 min = cal->min;
            s32 max = cal->max;
            s32 new_val;

            if (val > center) {
                new_val = (val - center) * JC_MAX_STICK_MAG;
                new_val /= (max - center);
            } else {
                new_val = (center - val) * -JC_MAX_STICK_MAG;
                new_val /= (center - min);
            }
            new_val = std::clamp(new_val, (s32)-JC_MAX_STICK_MAG, (s32)JC_MAX_STICK_MAG);
            return new_val;
        }

        static void joycon_read_stick_calibration(u8 *raw_cal,
                struct joycon_stick_cal_xy *cal,
                bool left_stick)
        {
            s32 x_max_above;
            s32 x_min_below;
            s32 y_max_above;
            s32 y_min_below;

            if (left_stick) {
                x_max_above = hid_field_extract((raw_cal + 0), 0, 12);
                x_min_below = hid_field_extract((raw_cal + 6), 0, 12);
                y_max_above = hid_field_extract((raw_cal + 1), 4, 12);
                y_min_below = hid_field_extract((raw_cal + 7), 4, 12);

                cal->x.center = hid_field_extract((raw_cal + 3), 0, 12);
                cal->y.center = hid_field_extract((raw_cal + 4), 4, 12);
            } else {
                cal->x.center = hid_field_extract((raw_cal + 0), 0, 12);
                cal->y.center = hid_field_extract((raw_cal + 1), 4, 12);
                x_min_below = hid_field_extract((raw_cal + 3), 0, 12);
                y_min_below = hid_field_extract((raw_cal + 4), 4, 12);
                x_max_above = hid_field_extract((raw_cal + 6), 0, 12);
                y_max_above = hid_field_extract((raw_cal + 7), 4, 12);
            }

            cal->x.max = cal->x.center + x_max_above;
            cal->x.min = cal->x.center - x_min_below;
            cal->y.max = cal->y.center + y_max_above;
            cal->y.min = cal->y.center - y_min_below;
        }

        void ConvertStickValues(SwitchAnalogStick *input, HidAnalogStickState  *output) {
            s32 x = static_cast<s32>(stick_ratio * (input->GetX() - STICK_CENTER));
            s32 y = static_cast<s32>(stick_ratio * (input->GetY() - STICK_CENTER));
            float mag = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
            if (mag < (0.12 * JOYSTICK_MAX)) {
                output->x = 0;
                output->y = 0;
            } else if (mag > (0.95 * JOYSTICK_MAX)) {
                float ratio = float(JOYSTICK_MAX) / mag;
                output->x = std::clamp<s32>(static_cast<s32>(x * ratio), JOYSTICK_MIN, JOYSTICK_MAX);
                output->y = std::clamp<s32>(static_cast<s32>(y * ratio), JOYSTICK_MIN, JOYSTICK_MAX);
            } else {
                output->x = std::clamp<s32>(x, JOYSTICK_MIN, JOYSTICK_MAX);
                output->y = std::clamp<s32>(y, JOYSTICK_MIN, JOYSTICK_MAX);
            }
        }

        const u8 led_player_mappings[] = {
            SwitchPlayerNumber_Unknown, //0000
            SwitchPlayerNumber_One,     //0001
            SwitchPlayerNumber_Unknown, //0010
            SwitchPlayerNumber_Two,     //0011
            SwitchPlayerNumber_Unknown, //0100
            SwitchPlayerNumber_Six,     //0101
            SwitchPlayerNumber_Eight,   //0110
            SwitchPlayerNumber_Three,   //0111
            SwitchPlayerNumber_One,     //1000
            SwitchPlayerNumber_Five,    //1001
            SwitchPlayerNumber_Six,     //1010
            SwitchPlayerNumber_Seven,   //1011
            SwitchPlayerNumber_Two,     //1100
            SwitchPlayerNumber_Seven,   //1101
            SwitchPlayerNumber_Three,   //1110
            SwitchPlayerNumber_Four,    //1111
        };

    }

    Result LedsMaskToPlayerNumber(u8 led_mask, u8 *player_number) {
        *player_number = led_player_mappings[(led_mask & 0xf) | (led_mask >> 4)];
        if (*player_number == SwitchPlayerNumber_Unknown) {
            return -1;
        }

        R_SUCCEED();
    }

    std::string GetControllerDirectory(const bluetooth::Address *address) {
        char path[0x100];
        util::SNPrintf(path, sizeof(path), "sdmc:/config/MissionControl/controllers/%02x%02x%02x%02x%02x%02x",
            address->address[0],
            address->address[1],
            address->address[2],
            address->address[3],
            address->address[4],
            address->address[5]
        );
        return path;
    }

    Result SwitchController::Initialize() {
        R_SUCCEED();
    }

    Result SwitchController::HandleDataReportEvent(const bluetooth::HidReportEventInfo *event_info) {
        const bluetooth::HidReport *report;
        if (hos::GetVersion() >= hos::Version_9_0_0) {
            report = &event_info->data_report.v9.report;
        } else if (hos::GetVersion() >= hos::Version_7_0_0) {
            report = reinterpret_cast<const bluetooth::HidReport *>(&event_info->data_report.v7.report);
        } else {
            report = reinterpret_cast<const bluetooth::HidReport *>(&event_info->data_report.v1.report);
        }

        if (!m_future_responses.empty()) {
            if ((m_future_responses.front()->GetType() == BtdrvHidEventType_Data) && (m_future_responses.front()->GetUserData() == report->data[0])) {
                m_future_responses.front()->SetData(*event_info);
            }
        }

        std::scoped_lock lk(m_input_mutex);

        this->UpdateControllerState(report);

        auto input_report = reinterpret_cast<SwitchInputReport *>(m_input_report.data);
        if (input_report->id == 0x21) {
            if (input_report->type0x21.hid_command_response.id == HidCommand_SerialFlashRead) {
                u8 *data = input_report->type0x21.hid_command_response.data.serial_flash_read.data;

                if (input_report->type0x21.hid_command_response.data.serial_flash_read.address == 0x6050) {
                    if (ams::mitm::GetSystemLanguage() == 10) {
                        u8 data[] = {0xff, 0xd7, 0x00, 0x00, 0x57, 0xb7, 0x00, 0x57, 0xb7, 0x00, 0x57, 0xb7};
                        std::memcpy(data, data, sizeof(data));
                    }
                }
                else if (input_report->type0x21.hid_command_response.data.serial_flash_read.address == 0x8010) {
                    m_has_user_cal_left = has_cal_magic(data);
                }
                else if (input_report->type0x21.hid_command_response.data.serial_flash_read.address == 0x801B) {
                    m_has_user_cal_right = has_cal_magic(data);
                }
                else if (input_report->type0x21.hid_command_response.data.serial_flash_read.address == 0x603D) {
                    joycon_read_stick_calibration(data, &m_cal_left, true);
                }
                else if (input_report->type0x21.hid_command_response.data.serial_flash_read.address == 0x6046) {
                    joycon_read_stick_calibration(data, &m_cal_right, false);
                }
                else if (input_report->type0x21.hid_command_response.data.serial_flash_read.address == 0x8012) {
                    joycon_read_stick_calibration(data, &m_cal_left_user, true);
                }
                else if (input_report->type0x21.hid_command_response.data.serial_flash_read.address == 0x801D) {
                    joycon_read_stick_calibration(data, &m_cal_right_user, false);
                }
            }
        }

        this->TranslateInputReport(input_report);
        this->ApplyButtonCombos(&input_report->buttons); 

        R_RETURN(bluetooth::hid::report::WriteHidDataReport(m_address, &m_input_report));
    }

    Result SwitchController::HandleSetReportEvent(const bluetooth::HidReportEventInfo *event_info) {
        if (!m_future_responses.empty()) {
            if (m_future_responses.front()->GetType() == BtdrvHidEventType_SetReport) {
                m_future_responses.front()->SetData(*event_info);
            }

            R_SUCCEED();
        }

        R_RETURN(bluetooth::hid::report::WriteHidSetReport(m_address, event_info->set_report.res));
    }

    Result SwitchController::HandleGetReportEvent(const bluetooth::HidReportEventInfo *event_info) {
        if (!m_future_responses.empty()) {
            if (m_future_responses.front()->GetType() == BtdrvHidEventType_GetReport) {
                m_future_responses.front()->SetData(*event_info);
            }

            R_SUCCEED();
        }

        auto report = hos::GetVersion() >= hos::Version_9_0_0 ? &event_info->get_report.v9.report : reinterpret_cast<const bluetooth::HidReport *>(&event_info->get_report.v1.report);
        R_RETURN(bluetooth::hid::report::WriteHidGetReport(m_address, report));
    }

    Result SwitchController::HandleOutputDataReport(const bluetooth::HidReport *report) {
        R_RETURN(this->WriteDataReport(report));
    }

    Result SwitchController::WriteDataReport(const bluetooth::HidReport *report) {
        R_RETURN(btdrvWriteHidData(m_address, report));
    }

    Result SwitchController::WriteDataReport(const bluetooth::HidReport *report, u8 response_id, bluetooth::HidReport *out_report) {       
        auto response = std::make_shared<HidResponse>(BtdrvHidEventType_Data);
        response->SetUserData(response_id);
        m_future_responses.push(response);
        ON_SCOPE_EXIT { m_future_responses.pop(); };

        R_TRY(btdrvWriteHidData(m_address, report));

        if (!response->TimedWait(ams::TimeSpan::FromMilliSeconds(500))) {
            return -1; // This should return a proper failure code
        }

        auto response_data = response->GetData();

        const bluetooth::HidReport *data_report;
        if (hos::GetVersion() >= hos::Version_9_0_0) {
            data_report = &response_data.data_report.v9.report;
        } else if (hos::GetVersion() >= hos::Version_7_0_0) {
            data_report = reinterpret_cast<const bluetooth::HidReport *>(&response_data.data_report.v7.report);
        } else {
            data_report = reinterpret_cast<const bluetooth::HidReport *>(&response_data.data_report.v1.report);
        }

        out_report->size = data_report->size;
        std::memcpy(&out_report->data, &data_report->data, data_report->size);

        R_SUCCEED();
    }

    Result SwitchController::SetReport(BtdrvBluetoothHhReportType type, const bluetooth::HidReport *report) {
        auto response = std::make_shared<HidResponse>(BtdrvHidEventType_SetReport);
        m_future_responses.push(response);
        ON_SCOPE_EXIT { m_future_responses.pop(); };

        R_TRY(btdrvSetHidReport(m_address, type, report));

        if (!response->TimedWait(ams::TimeSpan::FromMilliSeconds(500))) {
            return -1; // This should return a proper failure code
        }

        auto response_data = response->GetData();

        return response_data.set_report.res;
    }

    Result SwitchController::GetReport(u8 id, BtdrvBluetoothHhReportType type, bluetooth::HidReport *out_report) {
        auto response = std::make_shared<HidResponse>(BtdrvHidEventType_GetReport);
        m_future_responses.push(response);
        ON_SCOPE_EXIT { m_future_responses.pop(); };

        R_TRY(btdrvGetHidReport(m_address, id, type));

        if (!response->TimedWait(ams::TimeSpan::FromMilliSeconds(500))) {
            return -1; // This should return a proper failure code
        }

        auto response_data = response->GetData();
        
        Result result;
        const bluetooth::HidReport *get_report;
        if (hos::GetVersion() >= hos::Version_9_0_0) {
            result = response_data.get_report.v9.res;
            get_report = &response_data.get_report.v9.report;
        } else {
            result = response_data.get_report.v1.res;
            get_report = reinterpret_cast<const bluetooth::HidReport *>(&response_data.get_report.v1.report);
        }

        if (R_SUCCEEDED(result)) {
            out_report->size = get_report->size;
            std::memcpy(&out_report->data, &get_report->data, get_report->size);
        }

        return result;
    }

    void SwitchController::UpdateControllerState(const bluetooth::HidReport *report) {
        m_input_report.size = report->size;
        std::memcpy(m_input_report.data, report->data, report->size);
    }

    void SwitchController::TranslateInputReport(SwitchInputReport *input_report) {
        SwitchButtonData *buttons = &input_report->buttons;

        size_t hdls_id = SIZE_MAX;
        if (buttons->ZL) {
            hdls_id = 0;
        } else if(buttons->L) {
            hdls_id = 1;
        } else if (buttons->R) {
            hdls_id = 2;
        }

        if (buttons->lstick_press && buttons->rstick_press && hdls_id != SIZE_MAX) {
            struct hdls_controller *hdls_controller = &m_hdls_controllers[hdls_id];
            if (!m_hdls_combo_pressed) {
                if (hdls_controller->initialized) {
                    hiddbgDetachHdlsVirtualDevice(hdls_controller->handle);
                    hdls_controller->initialized = false;
                } else {
                    R_ABORT_UNLESS(hiddbgAttachHdlsVirtualDevice(&hdls_controller->handle, &hdls_controller->device_info));
                    R_ABORT_UNLESS(hiddbgSetHdlsState(hdls_controller->handle, &hdls_controller->state));
                    hdls_controller->initialized = true;
                }
            }

            m_hdls_combo_pressed = true;
        } else {
            m_hdls_combo_pressed = false;
        }

        for (size_t i=0; i<ARRAY_SIZE(m_hdls_controllers); i+=1) {
            struct hdls_controller *hdls_controller = &m_hdls_controllers[i];

            if (!hdls_controller->initialized) {
                continue;
            }

            hdls_controller->state.buttons = 0;
            hdls_controller->state.analog_stick_l.x = 0;
            hdls_controller->state.analog_stick_l.y = 0;

            if (i == hdls_id) {
                if (buttons->A) {
                    hdls_controller->state.buttons |= HidNpadButton_A;
                }
                if (buttons->B) {
                    hdls_controller->state.buttons |= HidNpadButton_B;
                }
                if (buttons->L) {
                    hdls_controller->state.buttons |= HidNpadButton_L;
                }
                if (buttons->R) {
                    hdls_controller->state.buttons |= HidNpadButton_R;
                }
                if (buttons->X) {
                    hdls_controller->state.buttons |= HidNpadButton_X;
                }
                if (buttons->Y) {
                    hdls_controller->state.buttons |= HidNpadButton_Y;
                }
                if (buttons->ZR) {
                    hdls_controller->state.buttons |= HidNpadButton_ZR;
                }
                if (buttons->plus) {
                    hdls_controller->state.buttons |= HidNpadButton_Plus;
                }
                if (buttons->minus) {
                    hdls_controller->state.buttons |= HidNpadButton_Minus;
                }
                if (buttons->dpad_left) {
                    hdls_controller->state.buttons |= HidNpadButton_Left;
                }
                if (buttons->dpad_right) {
                    hdls_controller->state.buttons |= HidNpadButton_Right;
                }
                if (buttons->dpad_up) {
                    hdls_controller->state.buttons |= HidNpadButton_Up;
                }
                if (buttons->dpad_down) {
                    hdls_controller->state.buttons |= HidNpadButton_Down;
                }
                if (buttons->lstick_press) {
                    hdls_controller->state.buttons |= HidNpadButton_StickL;
                }
                if (buttons->rstick_press) {
                    hdls_controller->state.buttons |= HidNpadButton_StickR;
                }

                {
                    struct joycon_stick_cal_xy *cal = &m_cal_left;

                    if (m_has_user_cal_left) {
                        cal = &m_cal_left_user;
                    }

                    hdls_controller->state.analog_stick_l.x = joycon_map_stick_val(&cal->x, input_report->left_stick.GetX());
                    hdls_controller->state.analog_stick_l.y = joycon_map_stick_val(&cal->y, input_report->left_stick.GetY());
                }

                {
                    struct joycon_stick_cal_xy *cal = &m_cal_right;

                    if (m_has_user_cal_right) {
                        cal = &m_cal_right_user;
                    }

                    hdls_controller->state.analog_stick_r.x = joycon_map_stick_val(&cal->x, input_report->right_stick.GetX());
                    hdls_controller->state.analog_stick_r.y = joycon_map_stick_val(&cal->y, input_report->right_stick.GetY());
                }

                if(0) {
                ConvertStickValues(&input_report->left_stick, &hdls_controller->state.analog_stick_l);
                ConvertStickValues(&input_report->right_stick, &hdls_controller->state.analog_stick_r);
                }

                *buttons = {0};
                input_report->left_stick.SetX(STICK_CENTER);
                input_report->left_stick.SetY(STICK_CENTER);
                input_report->right_stick.SetX(STICK_CENTER);
                input_report->right_stick.SetY(STICK_CENTER);
            }

            if (R_FAILED(hiddbgSetHdlsState(hdls_controller->handle, &hdls_controller->state))) {
                hiddbgDetachHdlsVirtualDevice(hdls_controller->handle);
                hdls_controller->initialized = false;
            }
        }
    }

    void SwitchController::ApplyButtonCombos(SwitchButtonData *buttons) {
        // Home combo = MINUS + DPAD_DOWN
        if (buttons->minus && buttons->dpad_down) {
            buttons->home = 1;
            buttons->minus = 0;
            buttons->dpad_down = 0;
        }

        // Capture combo = MINUS + DPAD_UP
        if (buttons->minus && buttons->dpad_up) {
            buttons->capture = 1;
            buttons->minus = 0;
            buttons->dpad_up = 0;
        }
    }

}
