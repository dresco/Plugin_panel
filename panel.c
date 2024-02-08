/*

  panel.c - Control panel support

  Part of grblHAL

  Copyright (c) 2021-2023 Jon Escombe

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "panel.h"

#if PANEL_ENABLE == 1 || PANEL_ENABLE == 2

#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/state_machine.h"
#include "../grbl/report.h"
#include "../grbl/nvs_buffer.h"
#include "../grbl/protocol.h"
#else
#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"
#endif

#if PANEL_ENABLE == 1 && !(MODBUS_ENABLE)
#error "This Control panel configuration requires the Modbus plugin to be enabled!"
#endif

#if PANEL_ENABLE == 2 && !(CANBUS_ENABLE)
#error "This Control panel configuration requires the CAN bus plugin to be enabled!"
#endif

static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;

static void processKeypad(uint16_t[]);
static void processEncoder(int);
static void processDisplayData(panel_displaydata_t *);

// Globals
static uint16_t grbl_state;
static uint8_t mpg_axis = 0;
static panel_jog_mode_t jog_mode = jog_mode_x10;

static const char* axis[] = { "X", "Y", "Z", "A", "B", "C", "U", "V" }; // do we need a 'null' axis to disable mpg control?
static const char* wcs_strings[] = { "G54", "G55", "G56", "G57", "G58", "G59", "G59.1", "G59.2", "G59.3" };

static uint16_t keydata[N_KEYDATAS] = { 0 };
static panel_encoder_data_t encoder_data[N_ENCODERS] = { 0 };

static char sys_cmd_buffer[LINE_BUFFER_SIZE];

/*
 * Start of settings specific code
 */

static nvs_address_t nvs_address;
static panel_settings_t panel_settings = { 0 };

static const setting_group_detail_t panel_groups [] = {
    { Group_Root, Group_Panel, "Control panel"}
};

static const char encoder_mode[] = "Unused,"
                                   "Spindle override,"
                                   "Feed override,"
                                   "Rapid override,"
                                   "MPG jog,"
                                   "X jog,"
                                   "Y jog,"
                                   "Z jog,"
#if N_AXIS > 3
                                   "A jog,"
#endif
#if N_AXIS > 4
                                   "B jog,"
#endif
#if N_AXIS > 5
                                   "C jog,"
#endif
#if N_AXIS > 6
                                   "U jog,"
#endif
#if N_AXIS > 7
                                   "V jog"
#endif
                                        ;

static const setting_detail_t panel_setting_detail[] = {
    { Setting_Panel_ModbusAddress, Group_Panel, "Control panel ModBus address", NULL, Format_Int8, "##0", NULL, "255", Setting_NonCore, &panel_settings.modbus_address, NULL, NULL },

    { Setting_Panel_UpdateInterval, Group_Panel, "Control panel update interval (ms)", NULL, Format_Int16, "###0", "25", "1000", Setting_NonCore, &panel_settings.update_interval, NULL , NULL },
    { Setting_Panel_SpindleSpeed, Group_Panel, "Control panel spindle start speed", NULL, Format_Int16, "####0", "1000", "24000", Setting_NonCore, &panel_settings.spindle_speed, NULL , NULL },

    { Setting_Panel_JogSpeed_x1, Group_Panel, "Control panel x1 jog speed", NULL, Format_Int16, "####0", "1", "10000", Setting_NonCore, &panel_settings.jog_speed_x1, NULL , NULL },
    { Setting_Panel_JogSpeed_x10, Group_Panel, "Control panel x10 jog speed", NULL, Format_Int16, "####0", "1", "10000", Setting_NonCore, &panel_settings.jog_speed_x10, NULL , NULL },
    { Setting_Panel_JogSpeed_x100, Group_Panel, "Control panel x100 jog speed", NULL, Format_Int16, "####0", "1", "10000", Setting_NonCore, &panel_settings.jog_speed_x100, NULL , NULL },
    { Setting_Panel_JogSpeed_Keypad, Group_Panel, "Control panel keypad jog speed", NULL, Format_Int16, "####0", "1", "10000", Setting_NonCore, &panel_settings.jog_speed_keypad, NULL , NULL },

    { Setting_Panel_JogDistance_x1, Group_Panel, "Control panel x1 jog distance", NULL, Format_Decimal, "###0", "0.001", "10", Setting_NonCore, &panel_settings.jog_distance_x1, NULL , NULL },
    { Setting_Panel_JogDistance_x10, Group_Panel, "Control panel x10 jog distance", NULL, Format_Decimal, "###0", "0.001", "10", Setting_NonCore, &panel_settings.jog_distance_x10, NULL , NULL },
    { Setting_Panel_JogDistance_x100, Group_Panel, "Control panel x100 jog distance", NULL, Format_Decimal, "###0", "0.001", "10", Setting_NonCore, &panel_settings.jog_distance_x100, NULL , NULL },
    { Setting_Panel_JogDistance_Keypad, Group_Panel, "Control panel keypad jog distance", NULL, Format_Decimal, "###0", "0.001", "10", Setting_NonCore, &panel_settings.jog_distance_keypad, NULL , NULL },

    { Setting_Panel_JogAccelRamp, Group_Panel, "Control panel keypad jog acceleration ramp", NULL, Format_Int8, "##0", "10", "100", Setting_NonCore, &panel_settings.jog_accel_ramp, NULL , NULL },

    { Setting_Panel_Encoder0_Mode, Group_Panel, "Control panel encoder #0 mode", NULL, Format_RadioButtons, encoder_mode, NULL, NULL, Setting_NonCore, &panel_settings.encoder_mode[0], NULL, NULL },
    { Setting_Panel_Encoder0_Cpd, Group_Panel, "Control panel encoder #0 counts per detent", NULL, Format_Int8,"#0", "1", "4", Setting_NonCore, &panel_settings.encoder_cpd[0], NULL, NULL },

    { Setting_Panel_Encoder1_Mode, Group_Panel, "Control panel encoder #1 mode", NULL, Format_RadioButtons, encoder_mode, NULL, NULL, Setting_NonCore, &panel_settings.encoder_mode[1], NULL, NULL },
    { Setting_Panel_Encoder1_Cpd, Group_Panel, "Control panel encoder #1 counts per detent", NULL, Format_Int8, "#0", "1", "4", Setting_NonCore, &panel_settings.encoder_cpd[1], NULL, NULL },

    { Setting_Panel_Encoder2_Mode, Group_Panel, "Control panel encoder #2 mode", NULL, Format_RadioButtons, encoder_mode, NULL, NULL, Setting_NonCore, &panel_settings.encoder_mode[2], NULL, NULL },
    { Setting_Panel_Encoder2_Cpd, Group_Panel, "Control panel encoder #2 counts per detent", NULL, Format_Int8, "#0", "1", "4", Setting_NonCore, &panel_settings.encoder_cpd[2], NULL, NULL },

    { Setting_Panel_Encoder3_Mode, Group_Panel, "Control panel encoder #3 mode", NULL, Format_RadioButtons, encoder_mode, NULL, NULL, Setting_NonCore, &panel_settings.encoder_mode[3], NULL, NULL },
    { Setting_Panel_Encoder3_Cpd, Group_Panel, "Control panel encoder #3 counts per detent", NULL, Format_Int8, "#0", "1", "4", Setting_NonCore, &panel_settings.encoder_cpd[3], NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS
static const setting_descr_t panel_settings_descr[] = {
        { Setting_Panel_JogDistance_Keypad, "The distance requested when keypad jogging. "
                                            "If a key is held down, a new jog request is repeated at each panel update interval." },
        { Setting_Panel_JogSpeed_Keypad, "The speed requested when keypad jogging. "
                                         "If a key is held down, a new jog request is repeated at each panel update interval." },
        { Setting_Panel_JogAccelRamp, "If a key is held down, keypad jogging will accelerate to the requested speed over this number of panel updates.\\n"
                                      "Note: for ModBus connections, the number of updates will be doubled, as inputs and outputs are interleaved." },
        { Setting_Panel_Encoder0_Cpd, "Encoder counts per detent. Typically this would be 1, 2, or 4, and would be configured to match the physical detents on the encoder." },
        { Setting_Panel_Encoder1_Cpd, "Encoder counts per detent. Typically this would be 1, 2, or 4, and would be configured to match the physical detents on the encoder." },
        { Setting_Panel_Encoder2_Cpd, "Encoder counts per detent. Typically this would be 1, 2, or 4, and would be configured to match the physical detents on the encoder." },
        { Setting_Panel_Encoder3_Cpd, "Encoder counts per detent. Typically this would be 1, 2, or 4, and would be configured to match the physical detents on the encoder." },
};
#endif

// Restore default settings and write to non volatile storage (NVS).
static void panel_settings_restore (void)
{
    //printf("panel_settings_restore()\n");

    panel_settings.modbus_address      = PANEL_DEFAULT_MODBUS_ADDRESS;
    panel_settings.update_interval     = PANEL_DEFAULT_UPDATE_INTERVAL;
    panel_settings.spindle_speed       = PANEL_DEFAULT_SPINDLE_SPEED;

    panel_settings.jog_speed_x1        = PANEL_DEFAULT_JOG_SPEED_X1;
    panel_settings.jog_speed_x10       = PANEL_DEFAULT_JOG_SPEED_X10;
    panel_settings.jog_speed_x100      = PANEL_DEFAULT_JOG_SPEED_X100;
    panel_settings.jog_speed_keypad    = PANEL_DEFAULT_JOG_SPEED_KEYPAD;

    panel_settings.jog_distance_x1     = PANEL_DEFAULT_JOG_DISTANCE_X1;
    panel_settings.jog_distance_x10    = PANEL_DEFAULT_JOG_DISTANCE_X10;
    panel_settings.jog_distance_x100   = PANEL_DEFAULT_JOG_DISTANCE_X100;
    panel_settings.jog_distance_keypad = PANEL_DEFAULT_JOG_DISTANCE_KEYPAD;

    panel_settings.jog_accel_ramp      = PANEL_DEFAULT_JOG_KEYPAD_RAMP;

    panel_settings.encoder_mode[0] = jog_mpg;
    panel_settings.encoder_cpd[0]  = 4;
    panel_settings.encoder_mode[1] = rapid_override;
    panel_settings.encoder_cpd[1]  = 4;
    panel_settings.encoder_mode[2] = spindle_override;
    panel_settings.encoder_cpd[2]  = 4;
    panel_settings.encoder_mode[3] = feed_override;
    panel_settings.encoder_cpd[3]  = 4;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&panel_settings, sizeof(panel_settings_t), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void panel_settings_load (void)
{
    //printf("panel_settings_load()\n");

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&panel_settings, nvs_address, sizeof(panel_settings_t), true) != NVS_TransferResult_OK) {
        panel_settings_restore();
    }
}

// Write settings to non volatile storage (NVS).
static void panel_settings_save (void)
{
    //printf("panel_settings_save()\n");

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&panel_settings, sizeof(panel_settings_t), true);
}

// Plugin settings have been changed.
void on_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    //printf("on_settings_changed()\n");

    for (uint8_t i=0; i < N_ENCODERS; i++) {
        encoder_data[i].mode = panel_settings.encoder_mode[i];
        encoder_data[i].cpd = panel_settings.encoder_cpd[i];
    }
}

static setting_details_t setting_details = {
    .groups = panel_groups,
    .n_groups = sizeof(panel_groups) / sizeof(setting_group_detail_t),
    .settings = panel_setting_detail,
    .n_settings = sizeof(panel_setting_detail) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = panel_settings_descr,
    .n_descriptions = sizeof(panel_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = panel_settings_save,
    .load = panel_settings_load,
    .restore = panel_settings_restore,
    .on_changed = on_settings_changed
};

/*
 * End of settings specific code
 */

#if PANEL_ENABLE == 1
static void rx_modbus_packet (modbus_message_t *msg);
static void rx_modbus_exception (uint8_t code, void *context);

static const modbus_callbacks_t modbus_callbacks = {
    .on_rx_packet = rx_modbus_packet,
    .on_rx_exception = rx_modbus_exception
};

static void ReadModbusInputRegisters(bool block)
{
    modbus_message_t read_cmd = {
        .context = (void *)Panel_ReadInputRegisters,
        .crc_check = true,
        .adu[0] = panel_settings.modbus_address,
        .adu[1] = ModBus_ReadInputRegisters,
        .adu[2] = 0x00,                                 // Start address   - high byte
        .adu[3] = PANEL_MODBUS_START_REG,               // Start address   - low byte - 100 (0x64)
        .adu[4] = 0x00,                                 // No of registers - high byte
        .adu[5] = PANEL_MODBUS_READREG_COUNT,           // No of registers - low byte
        .tx_length = 8,                                 // number of registers, plus 2 checksum bytes
        .rx_length = (2*PANEL_MODBUS_READREG_COUNT) + 5 // number of data registers requested,
                                                        // plus 3 header bytes (address, function, length),
                                                        // plus 2 checksum bytes
         // note: rx_length & tx_length must be less than or equal to MODBUS_MAX_ADU_SIZE
    };

    modbus_send(&read_cmd, &modbus_callbacks, block);
}

static void WriteModbusHoldingRegisters(bool block)
{
    static panel_displaydata_t displaydata;

    processDisplayData(&displaydata);

    modbus_message_t write_cmd = {
        .context = (void *)Panel_WriteHoldingRegisters,
        .crc_check = true,
        .adu[0] = panel_settings.modbus_address,
        .adu[1] = ModBus_WriteRegisters,
        .adu[2] = 0x00,                                         // Start address - high byte
        .adu[3] = PANEL_MODBUS_START_REG,                       // Start address - low byte - 100 (0x64)
        .adu[4] = 0x00,                                         // No of 16bit registers - high byte
        .adu[5] = PANEL_MODBUS_WRITEREG_COUNT,                  // No of 16bit registers - low byte
        .adu[6] = PANEL_MODBUS_WRITEREG_COUNT*2,                // Number of bytes

        .adu[7] = (displaydata.grbl_state >> 8) & 0xFF,         // Register 100 - high byte
        .adu[8] = displaydata.grbl_state & 0xFF,                // Register 100 - low byte

        .adu[11] = (displaydata.spindle_speed >> 8) & 0xFF,     // Register 102 - high byte
        .adu[12] = displaydata.spindle_speed & 0xFF,            // Register 102 - low byte

#if VFD_ENABLE
        .adu[13] = (displaydata.spindle_load >> 8) & 0xFF,      // Register 103 - high byte
        .adu[14] = displaydata.spindle_load & 0xFF,             // Register 103 - low byte
#endif

        .adu[15] = displaydata.wcs,                             // Register 104 - high byte
        .adu[16] = displaydata.spindle_override,                // Register 104 - low byte

        .adu[17] = displaydata.rapid_override,                  // Register 105 - high byte
        .adu[18] = displaydata.feed_override,                   // Register 105 - low byte

        .adu[19] = displaydata.mpg_mode,                        // Register 106 - high byte
        .adu[20] = displaydata.jog_mode,                        // Register 106 - low byte

        .adu[21] = displaydata.position[0].bytes[1],            // Register 107 - x position
        .adu[22] = displaydata.position[0].bytes[0],            // Register 107 - x position
        .adu[23] = displaydata.position[0].bytes[3],            // Register 108 - x position
        .adu[24] = displaydata.position[0].bytes[2],            // Register 108 - x position

        .adu[25] = displaydata.position[1].bytes[1],            // Register 109 - y position
        .adu[26] = displaydata.position[1].bytes[0],            // Register 109 - y position
        .adu[27] = displaydata.position[1].bytes[3],            // Register 110 - y position
        .adu[28] = displaydata.position[1].bytes[2],            // Register 110 - y position

        .adu[29] = displaydata.position[2].bytes[1],            // Register 111 - z position
        .adu[30] = displaydata.position[2].bytes[0],            // Register 111 - z position
        .adu[31] = displaydata.position[2].bytes[3],            // Register 112 - z position
        .adu[32] = displaydata.position[2].bytes[2],            // Register 112 - z position

        .tx_length = (2*PANEL_MODBUS_WRITEREG_COUNT) + 9,  // number of registers written, plus 7 header bytes, plus 2 checksum bytes
        .rx_length = 8                                     // fixed length ACK response?
        // note: rx_length & tx_length must be less than or equal to MODBUS_MAX_ADU_SIZE
    };

    modbus_send(&write_cmd, &modbus_callbacks, block);
}

static void rx_modbus_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((panel_modbus_response_t)msg->context) {

            case Panel_ReadInputRegisters:
                encoder_data[0].raw_value = (msg->adu[7] << 8)  | msg->adu[8];      // Register 102
                encoder_data[1].raw_value = (msg->adu[9] << 8)  | msg->adu[10];     // Register 103
                encoder_data[2].raw_value = (msg->adu[11] << 8) | msg->adu[12];     // Register 104
                encoder_data[3].raw_value = (msg->adu[13] << 8) | msg->adu[14];     // Register 105

                keydata[0] = (msg->adu[15] << 8) | msg->adu[16];                    // Register 106
                keydata[1] = (msg->adu[17] << 8) | msg->adu[18];                    // Register 107
                keydata[2] = (msg->adu[19] << 8) | msg->adu[20];                    // Register 108
                keydata[3] = (msg->adu[21] << 8) | msg->adu[22];                    // Register 109

                processKeypad(keydata);

                for (int i = 0; i < N_ENCODERS; i++) {
                    processEncoder(i);
                    // after first pass through, have populated the initial encoder values..
                    encoder_data[i].init_ok = true;
                }
                break;

            case Panel_WriteHoldingRegisters:
                break;

            default:
                break;
        }
    }

}

static void rx_modbus_exception (uint8_t code, void *context)
{
    // todo: need a 'Panel' alarm status
    system_raise_alarm(Alarm_None);
}
#endif // PANEL_ENABLE == 1

#if PANEL_ENABLE == 2
static dequeue_rx_ptr dequeue_rx;
static canbus_message_t tx_message;

bool panel_dequeue_rx (canbus_message_t message)
{
    // process inbound data here..
    //printf("panel_dequeue_rx(), CAN message id:%lx\n", message.id);

    // fixme: don't try accessing array elements that may not be allocated, respect N_KEYDATAS / N_ENCODERS etc..

    switch (message.id) {
        case CANBUS_PANEL_KEYPAD_1:
            keydata[0] = (message.data[0] << 8) | message.data[1];
            keydata[1] = (message.data[2] << 8) | message.data[3];
            keydata[2] = (message.data[4] << 8) | message.data[5];
            keydata[3] = (message.data[6] << 8) | message.data[7];

            processKeypad(keydata);
            break;

        case CANBUS_PANEL_KEYPAD_2:
            keydata[4] = (message.data[0] << 8) | message.data[1];
            keydata[5] = (message.data[2] << 8) | message.data[3];

            processKeypad(keydata);
            break;

        case CANBUS_PANEL_ENCODER_1:
            encoder_data[0].raw_value = (message.data[0] << 8) | message.data[1];
            encoder_data[1].raw_value = (message.data[2] << 8) | message.data[3];
            encoder_data[2].raw_value = (message.data[4] << 8) | message.data[5];
            encoder_data[3].raw_value = (message.data[6] << 8) | message.data[7];

            for (int i = 0; i < 4; i++) {
                if (encoder_data[i].function != unused) {
                    processEncoder(i);
                }
            }
            break;

        case CANBUS_PANEL_ENCODER_2:
            encoder_data[4].raw_value = (message.data[0] << 8) | message.data[1];
            encoder_data[5].raw_value = (message.data[2] << 8) | message.data[3];
            encoder_data[6].raw_value = (message.data[4] << 8) | message.data[5];
            encoder_data[7].raw_value = (message.data[6] << 8) | message.data[7];

            for (int i = 4; i < 8; i++) {
                if (encoder_data[i].function != unused) {
                    processEncoder(i);
                }
            }
            break;

        default:
            break;
    }

    /* Call the next rx handler in the chain */
    if (dequeue_rx)
        dequeue_rx(message);

    return(0);
}

void WriteCANbusOutputs()
{
    panel_displaydata_t displaydata;

    processDisplayData(&displaydata);

    // State
    memset(&tx_message, 0, sizeof(tx_message));
    tx_message.id = CANBUS_PANEL_STATE_1;
    tx_message.len = 8;
    tx_message.data[0] = 0;
    tx_message.data[1] = 0;
    tx_message.data[2] = (displaydata.grbl_state >> 8) & 0xFF;     // high byte
    tx_message.data[3] = displaydata.grbl_state & 0xFF;            // low byte
    tx_message.data[4] = (displaydata.spindle_speed >> 8) & 0xFF;  // high byte
    tx_message.data[5] = displaydata.spindle_speed & 0xFF;         // low byte
    tx_message.data[6] = (displaydata.spindle_load >> 8) & 0xFF;   // high byte
    tx_message.data[7] = displaydata.spindle_load & 0xFF;          // low byte
    canbus_queue_tx(tx_message);

    memset(&tx_message, 0, sizeof(tx_message));
    tx_message.id = CANBUS_PANEL_STATE_2;
    tx_message.len = 6;
    tx_message.data[0] = displaydata.spindle_override;
    tx_message.data[1] = displaydata.feed_override;
    tx_message.data[2] = displaydata.rapid_override;
    tx_message.data[3] = displaydata.wcs;
    tx_message.data[4] = displaydata.mpg_mode;
    tx_message.data[5] = displaydata.jog_mode;
    canbus_queue_tx(tx_message);

    // Machine position - up to 8 axis supported
    memset(&tx_message, 0, sizeof(tx_message));
    tx_message.id = CANBUS_PANEL_MPOS_1;
    tx_message.len = 8;
    tx_message.data[0] = (displaydata.position[0].bytes[1]);
    tx_message.data[1] = (displaydata.position[0].bytes[0]);
    tx_message.data[2] = (displaydata.position[0].bytes[3]);
    tx_message.data[3] = (displaydata.position[0].bytes[2]);
    tx_message.data[4] = (displaydata.position[1].bytes[1]);
    tx_message.data[5] = (displaydata.position[1].bytes[0]);
    tx_message.data[6] = (displaydata.position[1].bytes[3]);
    tx_message.data[7] = (displaydata.position[1].bytes[2]);
    canbus_queue_tx(tx_message);

    memset(&tx_message, 0, sizeof(tx_message));
    tx_message.id = CANBUS_PANEL_MPOS_2;
    tx_message.len = 4;
    tx_message.data[0] = (displaydata.position[2].bytes[1]);
    tx_message.data[1] = (displaydata.position[2].bytes[0]);
    tx_message.data[2] = (displaydata.position[2].bytes[3]);
    tx_message.data[3] = (displaydata.position[2].bytes[2]);
#if N_AXIS > 3
    tx_message.len = 8;
    tx_message.data[4] = (displaydata.position[3].bytes[1]);
    tx_message.data[5] = (displaydata.position[3].bytes[0]);
    tx_message.data[6] = (displaydata.position[3].bytes[3]);
    tx_message.data[7] = (displaydata.position[3].bytes[2]);
#endif
    canbus_queue_tx(tx_message);

#if N_AXIS > 4
    memset(&tx_message, 0, sizeof(tx_message));
    tx_message.id = CANBUS_PANEL_MPOS_3;
    tx_message.len = 4;
    tx_message.data[0] = (displaydata.position[4].bytes[1]);
    tx_message.data[1] = (displaydata.position[4].bytes[0]);
    tx_message.data[2] = (displaydata.position[4].bytes[3]);
    tx_message.data[3] = (displaydata.position[4].bytes[2]);
#if N_AXIS > 5
    tx_message.len = 8;
    tx_message.data[4] = (displaydata.position[5].bytes[1]);
    tx_message.data[5] = (displaydata.position[5].bytes[0]);
    tx_message.data[6] = (displaydata.position[5].bytes[3]);
    tx_message.data[7] = (displaydata.position[5].bytes[2]);
#endif
    canbus_queue_tx(tx_message);
#endif

#if N_AXIS > 4
    memset(&tx_message, 0, sizeof(tx_message));
    tx_message.id = CANBUS_PANEL_MPOS_4;
    tx_message.len = 4;
    tx_message.data[0] = (displaydata.position[4].bytes[1]);
    tx_message.data[1] = (displaydata.position[4].bytes[0]);
    tx_message.data[2] = (displaydata.position[4].bytes[3]);
    tx_message.data[3] = (displaydata.position[4].bytes[2]);
#if N_AXIS > 5
    tx_message.len = 8;
    tx_message.data[4] = (displaydata.position[5].bytes[1]);
    tx_message.data[5] = (displaydata.position[5].bytes[0]);
    tx_message.data[6] = (displaydata.position[5].bytes[3]);
    tx_message.data[7] = (displaydata.position[5].bytes[2]);
#endif
    canbus_queue_tx(tx_message);
#endif

#if N_AXIS > 6
    memset(&tx_message, 0, sizeof(tx_message));
    tx_message.id = CANBUS_PANEL_MPOS_5;
    tx_message.len = 4;
    tx_message.data[0] = (displaydata.position[6].bytes[1]);
    tx_message.data[1] = (displaydata.position[6].bytes[0]);
    tx_message.data[2] = (displaydata.position[6].bytes[3]);
    tx_message.data[3] = (displaydata.position[6].bytes[2]);
#if N_AXIS > 7
    tx_message.len = 8;
    tx_message.data[4] = (displaydata.position[7].bytes[1]);
    tx_message.data[5] = (displaydata.position[7].bytes[0]);
    tx_message.data[6] = (displaydata.position[7].bytes[3]);
    tx_message.data[7] = (displaydata.position[7].bytes[2]);
#endif
    canbus_queue_tx(tx_message);
#endif

}
#endif // PANEL_ENABLE == 2

static void processDisplayData(panel_displaydata_t *displaydata)
{
    static uint32_t last_ms;
    uint32_t ms = hal.get_elapsed_ticks();

    // Don't spam the spindle, retrieving the spindle state may generate multiple new modbus requests
    // (for instance Huanyang v1 uses seperate requests for RPM and Amps), which can saturate the
    // Modbus RX buffer if spindle is offline...
    //
    // todo: can we set this a bit more intelligently, based on a multiple of the modbus rx timeout?
    if (ms - last_ms > 250) {
        last_ms = ms;

        spindle_ptrs_t *spindle_0;
        spindle_state_t spindle_0_state;

        spindle_0 = spindle_get(0);
        spindle_0_state = spindle_0->get_state(spindle_0);

        if(!spindle_0->get_data) {
            displaydata->spindle_speed = lroundf(spindle_0_state.on ? spindle_0->param->rpm_overridden : 0);
        } else {
            displaydata->spindle_speed = lroundf(spindle_0->get_data(SpindleData_RPM)->rpm);
        }

#if VFD_ENABLE
        const vfd_ptrs_t *vfd_spindle = vfd_get_active();
        if (vfd_spindle && vfd_spindle->get_load) {
            displaydata->spindle_load = lroundf(vfd_spindle->get_load());
        }
#endif

        displaydata->spindle_override = spindle_0->param->override_pct;
    }

    displaydata->grbl_state = grbl_state;

    displaydata->wcs = gc_state.modal.coord_system.id;

    displaydata->mpg_mode = mpg_axis;
    displaydata->jog_mode = jog_mode;

    displaydata->feed_override    = sys.override.feed_rate;
    displaydata->rapid_override   = sys.override.rapid_rate;

    int32_t raw_position[N_AXIS];
    float   machine_position[N_AXIS];

    memcpy(raw_position, sys.position, sizeof(sys.position));
    system_convert_array_steps_to_mpos(machine_position, raw_position);

    float wco[N_AXIS];
    for (uint_fast8_t idx = 0; idx < N_AXIS; idx++) {
        // Apply work coordinate offsets and tool length offset to current position.
        wco[idx] = gc_get_offset(idx);
        displaydata->position[idx].value = machine_position[idx] - wco[idx];
    }
}

static void processKeypad(uint16_t keydata[])
{
    static uint16_t last_keydata_1, last_keydata_2, last_keydata_3, last_keydata_4, last_keydata_5;
    char command[30] = "";
    bool jogRequested = false;
    static bool jogInProgress;
    static uint8_t jogRampCount;
    uint8_t keypad_jog_mode = jog_mode_smooth;

    panel_keydata_1_t keydata_1;
    panel_keydata_2_t keydata_2;
    panel_keydata_3_t keydata_3;
    panel_keydata_4_t keydata_4;
    panel_keydata_5_t keydata_5;

    keydata_1.value = keydata[0];
    keydata_2.value = keydata[1];
    keydata_3.value = keydata[2];
    keydata_4.value = keydata[3];
    keydata_5.value = keydata[4];

    UNUSED(keydata_5);
    UNUSED(last_keydata_5);

    //
    // keydata_1
    // - key repeats not required
    // - cycle start/feed hold/stop/reset can be executed in any state
    // - unlock only in locked state
    // - home only in idle state
    // - spindle control only in idle state
    // - single block toggle in idle/hold?
    //
    if (keydata_1.value != last_keydata_1) {

        // realtime commands - can be processed in any state
        if (keydata_1.stop)
            grbl.enqueue_realtime_command(CMD_STOP);

        if (keydata_1.feed_hold)
            grbl.enqueue_realtime_command(CMD_FEED_HOLD);

        if (keydata_1.cycle_start)
            grbl.enqueue_realtime_command(CMD_CYCLE_START);

        if (keydata_1.reset)
            grbl.enqueue_realtime_command(CMD_RESET);

        // change active mpg axis - can be processed in any state
        if (keydata_1.mpg_axis_x)
            mpg_axis = 0;
        if (keydata_1.mpg_axis_y)
            mpg_axis = 1;
        if (keydata_1.mpg_axis_z)
            mpg_axis = 2;
        if (keydata_1.mpg_axis_a)
            mpg_axis = 3;
        if (keydata_1.mpg_axis_b)
            mpg_axis = 4;

        // spindle keys - only from idle state
        if (grbl_state == STATE_IDLE) {
            if (keydata_1.spindle_off)
                grbl.enqueue_gcode("M5");
            if (keydata_1.spindle_cw) {
                strcat(command, "M3 S");
                strcat(command, ftoa(panel_settings.spindle_speed, 0));
                grbl.enqueue_gcode(command);
            }
            if (keydata_1.spindle_ccw) {
                strcat(command, "M4 S");
                strcat(command, ftoa(panel_settings.spindle_speed, 0));
                grbl.enqueue_gcode(command);
            }
        }

        // single block toggle - process only in IDLE or HOLD state
        // todo: think a bit more about what states this should be allowed in?
        //       need to reflect the current state on the display..
        if (keydata_1.single_block) {
            if (grbl_state == STATE_IDLE || (grbl_state & STATE_HOLD))
                grbl.enqueue_gcode("$S");
        }

        // unlock - only from alarm state
        // note: protocol_enqueue_gcode() doesn't accept input in alarm state - use system_execute_line() instead
        if (keydata_1.unlock) {
            if (grbl_state == STATE_ALARM) {
                strcpy(sys_cmd_buffer, "$X");
                system_execute_line((char *)sys_cmd_buffer);
            }
        }

        // home - only from idle state
        if (keydata_1.home) {
            if (grbl_state == STATE_IDLE) {
                strcpy(command, "$H");
                grbl.enqueue_gcode((char *)command);
            }
        }

    }
    last_keydata_1 = keydata_1.value;

    //
    // keydata_2
    // - key repeats not required
    // - set wcs, zero work offsets, move to zero - only from idle state
    //
    if (keydata_2.value != last_keydata_2) {

        if (grbl_state == STATE_IDLE) {

            // need to know the current WCS in order to set
            uint8_t wcs = gc_state.modal.coord_system.id;

            if (keydata_2.wcs_g54)
                grbl.enqueue_gcode("G54");
            if (keydata_2.wcs_g55)
                grbl.enqueue_gcode("G55");
            if (keydata_2.wcs_g56)
                grbl.enqueue_gcode("G56");
            if (keydata_2.wcs_g57)
                grbl.enqueue_gcode("G57");

            if (keydata_2.move_to_zero_x)
                grbl.enqueue_gcode("G0 X0");
            if (keydata_2.move_to_zero_y)
                grbl.enqueue_gcode("G0 Y0");
            if (keydata_2.move_to_zero_z)
                grbl.enqueue_gcode("G0 Z0");
            if (keydata_2.move_to_zero_a)
                grbl.enqueue_gcode("G0 A0");

            if (keydata_2.zero_work_offset_x) {
                strcpy(command, "G10 L20 ");
                strcat(command, wcs_strings[wcs]);
                strcat(command," X0");
                grbl.enqueue_gcode(command);
            } else if (keydata_2.zero_work_offset_y) {
                strcpy(command, "G10 L20 ");
                strcat(command, wcs_strings[wcs]);
                strcat(command," Y0");
                grbl.enqueue_gcode(command);
            } else if (keydata_2.zero_work_offset_z) {
                strcpy(command, "G10 L20 ");
                strcat(command, wcs_strings[wcs]);
                strcat(command," Z0");
                grbl.enqueue_gcode(command);
            } else if (keydata_2.zero_work_offset_a) {
                strcpy(command, "G10 L20 ");
                strcat(command, wcs_strings[wcs]);
                strcat(command," A0");
                grbl.enqueue_gcode(command);
            }
        }

    }
    last_keydata_2 = keydata_2.value;

    //
    // keydata_3
    // - update mpg jog mode from any state
    // - jogging ony in jog or idle states
    //

    // update mpg jog mode from any state
    if (keydata_3.value != last_keydata_3) {
        if (keydata_3.jog_step_x1)
            jog_mode = jog_mode_x1;
        if (keydata_3.jog_step_x10)
            jog_mode = jog_mode_x10;
        if (keydata_3.jog_step_x100)
            jog_mode = jog_mode_x100;
        if (keydata_3.jog_step_smooth)
            jog_mode = jog_mode_smooth;
    }

    // check for jogging key states even without change, as we want to keep jogging while pressed

    // only jog in idle or an existing jog state
    bool jogOkay = (grbl_state == STATE_IDLE || (grbl_state & STATE_JOG));

    if (jogOkay)
    {
        if (keydata_3.jog_positive_x) {
            strcpy(command, "$J=G91X");
            jogRequested = true;
        } else if (keydata_3.jog_negative_x) {
            strcpy(command, "$J=G91X-");
            jogRequested = true;
        } else if (keydata_3.jog_positive_y) {
            strcpy(command, "$J=G91Y");
            jogRequested = true;
        } else if (keydata_3.jog_negative_y) {
            strcpy(command, "$J=G91Y-");
            jogRequested = true;
        } else if (keydata_3.jog_positive_z) {
            strcpy(command, "$J=G91Z");
            jogRequested = true;
        } else if (keydata_3.jog_negative_z) {
            strcpy(command, "$J=G91Z-");
            jogRequested = true;
        } else if (keydata_3.jog_positive_a) {
            strcpy(command, "$J=G91A");
            jogRequested = true;
        } else if (keydata_3.jog_negative_a) {
            strcpy(command, "$J=G91A-");
            jogRequested = true;
        } else if (keydata_3.jog_positive_b) {
            strcpy(command, "$J=G91B");
            jogRequested = true;
        } else if (keydata_3.jog_negative_b) {
            strcpy(command, "$J=G91B-");
            jogRequested = true;
        }

        if (jogRequested && !plan_check_full_buffer())
        {
            // note: keypad jogging is currently always in smooth mode..
            switch (keypad_jog_mode) {

                case (jog_mode_x1):
                    strcat(command, ftoa(panel_settings.jog_distance_x1, 3));
                    strcat(command, "F");
                    strcat(command, ftoa(panel_settings.jog_speed_x1, 0));
                    break;

                case (jog_mode_x10):
                    strcat(command, ftoa(panel_settings.jog_distance_x10, 3));
                    strcat(command, "F");
                    strcat(command, ftoa(panel_settings.jog_speed_x10, 0));
                    break;

                case (jog_mode_x100):
                    strcat(command, ftoa(panel_settings.jog_distance_x100, 3));
                    strcat(command, "F");
                    strcat(command, ftoa(panel_settings.jog_speed_x100, 0));
                    break;

                case (jog_mode_smooth):
                    // Initial attempt at acceleration ramp for smooth keypad jogging..
                    if (!jogInProgress)
                        jogRampCount = panel_settings.jog_accel_ramp;
                    else if (jogRampCount)
                        jogRampCount--;

                    float jogAccel = (panel_settings.jog_accel_ramp - jogRampCount) / (float)panel_settings.jog_accel_ramp;

                    strcat(command, ftoa(panel_settings.jog_distance_keypad * jogAccel, 3));
                    strcat(command, "F");
                    strcat(command, ftoa(panel_settings.jog_speed_keypad * jogAccel, 0));
                    break;

                default:
                     break;

            }
            // don't repeat jog commands if in single step mode
            if ((keypad_jog_mode == jog_mode_smooth || !jogInProgress))
                jogInProgress = grbl.enqueue_gcode((char *)command);
        }
        // cancel jog immediately key released if smooth jogging
        if ((!jogRequested) && (keypad_jog_mode == jog_mode_smooth) && jogInProgress)
        {
            grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
            jogInProgress = false;
            jogRampCount = panel_settings.jog_accel_ramp;
        }

        // set jogInProgress back to 0 at end of move in single-step
        if ((!jogRequested) && (grbl_state == STATE_IDLE) && jogInProgress)
            jogInProgress = false;

    }
    last_keydata_3 = keydata_3.value;

    //
    // keydata_4
    // - key repeats not required
    // - overrides & resets from any state
    //
    if (keydata_4.value != last_keydata_4) {
        if (keydata_4.feed_override_reset)
            grbl.enqueue_realtime_command(CMD_OVERRIDE_FEED_RESET);
        if (keydata_4.spindle_override_reset)
            grbl.enqueue_realtime_command(CMD_OVERRIDE_SPINDLE_RESET);
        if (keydata_4.rapid_override_100)
            grbl.enqueue_realtime_command(CMD_OVERRIDE_RAPID_RESET);

    }
    last_keydata_4 = keydata_4.value;

}

static void processEncoderOverride(uint8_t encoder_index)
{
    int16_t signed_value;
    uint16_t cmd_override_minus = 0, cmd_override_plus = 0;
    int8_t modulo;

    switch (encoder_data[encoder_index].mode) {
        case (spindle_override):
            cmd_override_minus = CMD_OVERRIDE_SPINDLE_FINE_MINUS;
            cmd_override_plus  = CMD_OVERRIDE_SPINDLE_FINE_PLUS;
            break;

        case (feed_override):
            cmd_override_minus = CMD_OVERRIDE_FEED_FINE_MINUS;
            cmd_override_plus  = CMD_OVERRIDE_FEED_FINE_PLUS;
            break;

        case (rapid_override):
            break;

        default:
            return;

    }

    signed_value = encoder_data[encoder_index].raw_value - encoder_data[encoder_index].last_raw_value;
    signed_value = signed_value / encoder_data[encoder_index].cpd;

    modulo = encoder_data[encoder_index].raw_value % encoder_data[encoder_index].cpd;
    if (modulo && signed_value < 0) {
        modulo = (encoder_data[encoder_index].cpd - modulo) * -1;
    }

    // don't do any overrides if not initialised, just store the initial reading
    if (!encoder_data[encoder_index].init_ok) {
        encoder_data[encoder_index].last_raw_value = encoder_data[encoder_index].raw_value;
        return;
    }

    if (signed_value) {

        if (encoder_data[encoder_index].mode == rapid_override) {

            // rapid overrides are handled a bit differently, as only thee possible values..
            if (signed_value < 0) {
                switch (sys.override.rapid_rate) {

                    case RAPID_OVERRIDE_LOW:
                        break;

                    case RAPID_OVERRIDE_MEDIUM:
                        grbl.enqueue_realtime_command(CMD_OVERRIDE_RAPID_LOW);
                        break;

                    case DEFAULT_RAPID_OVERRIDE:
                        grbl.enqueue_realtime_command(CMD_OVERRIDE_RAPID_MEDIUM);
                        break;

                    default:
                        break;
                }
            } else {
                switch (sys.override.rapid_rate) {

                    case RAPID_OVERRIDE_LOW:
                        grbl.enqueue_realtime_command(CMD_OVERRIDE_RAPID_MEDIUM);
                        break;

                    case RAPID_OVERRIDE_MEDIUM:
                        grbl.enqueue_realtime_command(CMD_OVERRIDE_RAPID_RESET);
                        break;

                    case DEFAULT_RAPID_OVERRIDE:
                        break;

                    default:
                        break;
                }
            }

        } else {

            uint16_t count = abs(signed_value);
            bool is_negative = false;
            if (signed_value < 0)
                is_negative = true;

            for (uint16_t i = 0 ; i < count; i++) {
                if (is_negative)
                    grbl.enqueue_realtime_command(cmd_override_minus);
                else
                    grbl.enqueue_realtime_command(cmd_override_plus);
            }
        }

        // update last value
        // note stored value is adjusted for partial ticks
        encoder_data[encoder_index].last_raw_value = encoder_data[encoder_index].raw_value - modulo;
    }
}

static void processEncoderJog(uint8_t encoder_index, uint8_t jog_axis)
{

    int16_t signed_value;
    char command[30] = "";
    bool jogOkay = (grbl_state == STATE_IDLE || (grbl_state & STATE_JOG));
    int8_t modulo;

    signed_value = encoder_data[encoder_index].raw_value - encoder_data[encoder_index].last_raw_value;
    signed_value = signed_value / encoder_data[encoder_index].cpd;

    modulo = encoder_data[encoder_index].raw_value % encoder_data[encoder_index].cpd;
    if (modulo && signed_value < 0) {
        modulo = (encoder_data[encoder_index].cpd - modulo) * -1;
    }

    // don't jog if not initialised - just store the initial reading (so we can't pick up a big jump on startup)
    // don't jog if in smooth mode - is meant for keypad jogging only (large distances requested, and cancelled on key release)
    if (!encoder_data[encoder_index].init_ok || (jog_mode == jog_mode_smooth)) {
        encoder_data[encoder_index].last_raw_value = encoder_data[encoder_index].raw_value;
        return;
    }

    if (signed_value && jogOkay) {
        strcpy(command, "$J=G91");
        strcat(command, axis[jog_axis]);

        switch (jog_mode) {

            case (jog_mode_x1):
                strcat(command, ftoa(signed_value * panel_settings.jog_distance_x1, 3));
                strcat(command, "F");
                strcat(command, ftoa(panel_settings.jog_speed_x1, 0));
                break;

            case (jog_mode_x10):
                strcat(command, ftoa(signed_value * panel_settings.jog_distance_x10, 3));
                strcat(command, "F");
                strcat(command, ftoa(panel_settings.jog_speed_x10, 0));
                break;

            case (jog_mode_x100):
                strcat(command, ftoa(signed_value * panel_settings.jog_distance_x100, 3));
                strcat(command, "F");
                strcat(command, ftoa(panel_settings.jog_speed_x100, 0));
                break;

            default:
                 break;

        }

        if (!plan_check_full_buffer()) {
            if (grbl.enqueue_gcode((char *)command)) {
                // update last value, and only if jog command was accepted
                // note stored value is adjusted for partial ticks
                encoder_data[encoder_index].last_raw_value = encoder_data[encoder_index].raw_value - modulo;
            }
        }
    }
}

static void processEncoder(int index)
{
    switch (encoder_data[index].mode) {
        case (jog_mpg):
            processEncoderJog(index, mpg_axis);
            break;

        case (jog_x):
            processEncoderJog(index, 0);
            break;

        case (jog_y):
            processEncoderJog(index, 1);
            break;

        case (jog_z):
            processEncoderJog(index, 2);
            break;

        case (jog_a):
            processEncoderJog(index, 3);
            break;

        case (jog_b):
            processEncoderJog(index, 4);
            break;

        case (jog_c):
            processEncoderJog(index, 5);
            break;

        case (jog_u):
            processEncoderJog(index, 6);
            break;

        case (jog_v):
            processEncoderJog(index, 7);
            break;

        case (feed_override):
        case (spindle_override):
        case (rapid_override):
            processEncoderOverride(index);
            break;

        default:
            break;

    }
    // after first pass through, have populated the initial encoder values..
    encoder_data[index].init_ok = true;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:PANEL v0.02]" ASCII_EOL);
    }
}

void ReadPanelInputs(void)
{
#if PANEL_ENABLE == 1
    ReadModbusInputRegisters(false);      // do not block for modbus response
#endif

#if PANEL_ENABLE == 2
    // Process inputs from CAN bus plugin
#endif
}

void WritePanelOutputs(void)
{
#if PANEL_ENABLE == 1
    WriteModbusHoldingRegisters(false);   // do not block for modbus response
#endif

#if PANEL_ENABLE == 2
    // Send outputs to CAN bus plugin
    WriteCANbusOutputs();
#endif
}

void panel_update (sys_state_t state)
{
    static uint32_t last_ms;
    static bool write = false;

    // save into global variables for other functions to access the latest state..
    grbl_state = state;

    on_execute_realtime(state);

    uint32_t ms = hal.get_elapsed_ticks();

    if(ms == last_ms) // Don't check more than once every ms
        return;

    // Initiate requests to the panel every PANEL_UPDATE_INTERVAL ms, alternating inputs (buttons/encoders) and outputs (display)
    //
    // CAN bus - not using remote frames (request/response), so they will just be processed as received? (callback from canbus plugin)
    //
    // Need to consider the flow, modbus code expects to receive all data at once, and then process all...
    // what about overwriting values etc.. can we just process each message individually?
    //
    //
    if (!(ms % panel_settings.update_interval) )
    {
        if (!write)
            ReadPanelInputs();
        else
            WritePanelOutputs();

        write = !write;
    }

    last_ms = ms;
}

int plugins_enabled(void)
{
    int res = false;

#if MODBUS_ENABLE
    res = modbus_enabled();
#endif

#if CANBUS_ENABLE
    res = canbus_enabled();
#endif

    return res;
}

static void cancel_jog (sys_state_t state)
{
}

void panel_init()
{
    if(plugins_enabled()) {
        if ((nvs_address = nvs_alloc(sizeof(panel_settings_t)))) {

            settings_register(&setting_details);

            on_report_options = grbl.on_report_options;
            grbl.on_report_options = onReportOptions;

            on_execute_realtime = grbl.on_execute_realtime;
            grbl.on_execute_realtime = panel_update;

            grbl.on_jog_cancel = cancel_jog;

    #if PANEL_ENABLE == 2
            dequeue_rx = canbus.dequeue_rx;
            canbus.dequeue_rx = panel_dequeue_rx;
    #endif

        }
    }
}
#endif
