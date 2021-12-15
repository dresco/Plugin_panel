/*

  panel.c - Control panel support

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io
  Copyright (c) 2021 Jon Escombe

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

#if PANEL_ENABLE

#include "panel.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/state_machine.h"
#include "../grbl/report.h"
#else
#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/report.h"
#endif

static settings_changed_ptr settings_changed;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime;

// Globals
static bool init_ok = false;
static uint16_t grbl_state;
static uint8_t mpg_axis = 0;
static panel_jog_mode_t jog_mode = jog_mode_x10;

static const char* axis[] = { "X", "Y", "Z", "A", "B" }; // do we need a 'null' axis to disable mpg control?

static panel_encoder_data_t encoder_data[N_ENCODERS] = { 0 };

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};

static void ReadInputRegisters(bool block)
{
    modbus_message_t read_cmd = {
        .context = (void *)Panel_ReadInputRegisters,
        .crc_check = true,
        .adu[0] = PANEL_ADDRESS,
        .adu[1] = ModBus_ReadInputRegisters,
        .adu[2] = 0x00,                          // Start address   - high byte
        .adu[3] = PANEL_START_REF,               // Start address   - low byte - 100 (0x64)
        .adu[4] = 0x00,                          // No of registers - high byte
        .adu[5] = PANEL_READREG_COUNT,           // No of registers - low byte
        .tx_length = 8,                          // number of registers, plus 2 checksum bytes
        .rx_length = (2*PANEL_READREG_COUNT) + 5 // number of data registers requested,
                                                 // plus 3 header bytes (address, function, length),
                                                 // plus 2 checksum bytes
         // note: rx_length & tx_length must be less than or equal to MODBUS_MAX_ADU_SIZE
    };

    modbus_send(&read_cmd, &callbacks, block);
}

static void WriteHoldingRegisters(bool block)
{

    uint16_t spindle_rpm;
    spindle_state_t sp_state = hal.spindle.get_state();

    if(!hal.spindle.get_data) {
        spindle_rpm = lroundf(sp_state.on ? sys.spindle_rpm : 0);
    } else {
        spindle_rpm = lroundf(hal.spindle.get_data(SpindleData_RPM)->rpm);
    }

    uint8_t spindle_override = sys.override.spindle_rpm;
    uint8_t feed_override    = sys.override.feed_rate;
    uint8_t rapid_override   = sys.override.rapid_rate;
    uint8_t wcs = gc_state.modal.coord_system.id;

    int32_t raw_position[N_AXIS];
    float   machine_position[N_AXIS];
    float32_data_t offset_position[N_AXIS];

    memcpy(raw_position, sys.position, sizeof(sys.position));
    system_convert_array_steps_to_mpos(machine_position, raw_position);

    float wco[N_AXIS];
    for (uint_fast8_t idx = 0; idx < N_AXIS; idx++) {
        // Apply work coordinate offsets and tool length offset to current position.
        wco[idx] = gc_get_offset(idx);
        offset_position[idx].value = machine_position[idx] - wco[idx];
    }

    modbus_message_t write_cmd = {
        .context = (void *)Panel_WriteHoldingRegisters,
        .crc_check = true,
        .adu[0] = PANEL_ADDRESS,
        .adu[1] = ModBus_WriteRegisters,
        .adu[2] = 0x00,                             // Start address - high byte
        .adu[3] = PANEL_START_REF,                  // Start address - low byte - 100 (0x64)
        .adu[4] = 0x00,                             // No of 16bit registers - high byte
        .adu[5] = PANEL_WRITEREG_COUNT,             // No of 16bit registers - low byte
        .adu[6] = PANEL_WRITEREG_COUNT*2,           // Number of bytes

        .adu[7] = (grbl_state >> 8) & 0xFF,         // Register 100 - high byte
        .adu[8] = grbl_state & 0xFF,                // Register 100 - low byte

        .adu[11] = (spindle_rpm >> 8) & 0xFF,       // Register 102 - high byte
        .adu[12] = spindle_rpm & 0xFF,              // Register 102 - low byte

#if SPINDLE_POWER
        .adu[13] = (spindle_power >> 8) & 0xFF,     // Register 103 - high byte
        .adu[14] = spindle_power & 0xFF,            // Register 103 - low byte
#endif

        .adu[15] = wcs,                             // Register 104 - high byte
        .adu[16] = spindle_override,                // Register 104 - low byte

        .adu[17] = rapid_override,                  // Register 105 - high byte
        .adu[18] = feed_override,                   // Register 105 - low byte

        .adu[19] = mpg_axis,                        // Register 106 - high byte
        .adu[20] = jog_mode,                        // Register 106 - low byte

        .adu[21] = offset_position[0].bytes[1],     // Register 107 - x position
        .adu[22] = offset_position[0].bytes[0],     // Register 107 - x position
        .adu[23] = offset_position[0].bytes[3],     // Register 108 - x position
        .adu[24] = offset_position[0].bytes[2],     // Register 109 - x position

        .adu[25] = offset_position[1].bytes[1],     // Register 109 - y position
        .adu[26] = offset_position[1].bytes[0],     // Register 109 - y position
        .adu[27] = offset_position[1].bytes[3],     // Register 110 - y position
        .adu[28] = offset_position[1].bytes[2],     // Register 110 - y position

        .adu[29] = offset_position[2].bytes[1],     // Register 111 - z position
        .adu[30] = offset_position[2].bytes[0],     // Register 111 - z position
        .adu[31] = offset_position[2].bytes[3],     // Register 112 - z position
        .adu[32] = offset_position[2].bytes[2],     // Register 112 - z position

        .tx_length = (2*PANEL_WRITEREG_COUNT) + 9,  // number of registers written, plus 7 header bytes, plus 2 checksum bytes
        .rx_length = 8                              // fixed length ACK response?
        // note: rx_length & tx_length must be less than or equal to MODBUS_MAX_ADU_SIZE
    };

    modbus_send(&write_cmd, &callbacks, block);
}

static void processKeypad(uint16_t keydata[])
{
    static uint16_t last_keydata_1, last_keydata_2, last_keydata_3, last_keydata_4, last_keydata_5;
    char command[30] = "";
    bool jogRequested = false;
    static bool jogInProgress;

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

    UNUSED(keydata_2);
    UNUSED(keydata_4);
    UNUSED(keydata_5);
    UNUSED(last_keydata_2);
    UNUSED(last_keydata_4);
    UNUSED(last_keydata_5);

    //
    // keydata_1
    // - key repeats not required
    //
    if (keydata_1.value != last_keydata_1) {
        if (keydata_1.stop)
            grbl.enqueue_realtime_command(CMD_STOP);

        if (keydata_1.feed_hold)
            grbl.enqueue_realtime_command(CMD_FEED_HOLD);

        if (keydata_1.cycle_start)
            grbl.enqueue_realtime_command(CMD_CYCLE_START);

        if (keydata_1.reset)
            grbl.enqueue_realtime_command(CMD_RESET);

        if (keydata_1.unlock) {
            strcpy(command, "$X");
            grbl.enqueue_gcode((char *)command);
        }

        if (keydata_1.home) {
            strcpy(command, "$H");
            grbl.enqueue_gcode((char *)command);
        }

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
    }
    last_keydata_1 = keydata_1.value;

    //
    // keydata_3
    // todo: add support for diagonal moves?
    //
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
            switch (jog_mode) {

                case (jog_mode_x1):
                    strcat(command, ftoa(JOG_DISTANCE_X1, 3));
                    strcat(command, "F");
                    strcat(command, ftoa(JOG_SPEED_X1, 0));
                    break;

                case (jog_mode_x10):
                    strcat(command, ftoa(JOG_DISTANCE_X10, 3));
                    strcat(command, "F");
                    strcat(command, ftoa(JOG_SPEED_X10, 0));
                    break;

                case (jog_mode_x100):
                    strcat(command, ftoa(JOG_DISTANCE_X100, 3));
                    strcat(command, "F");
                    strcat(command, ftoa(JOG_SPEED_X100, 0));
                    break;

                // todo: add a smooth acceleration ramp for this one..?
                case (jog_mode_smooth):
                    strcat(command, ftoa(JOG_DISTANCE_X10, 3));
                    strcat(command, "F");
                    strcat(command, ftoa(JOG_SPEED_X10, 0));
                    break;

                default:
                     break;

            }
            // don't repeat jog commands if in single step mode
            if ((jog_mode == jog_mode_smooth || !jogInProgress))
                jogInProgress = grbl.enqueue_gcode((char *)command);
        }
        // cancel jog immediately key released if smooth jogging
        if ((!jogRequested) && (jog_mode == jog_mode_smooth) && jogInProgress)
        {
            grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
            jogInProgress = false;
        }

        // set jogInProgress back to 0 at end of move in single-step
        if ((!jogRequested) && (grbl_state == STATE_IDLE) && jogInProgress)
            jogInProgress = false;

    }
    last_keydata_3 = keydata_3.value;
}

static void processEncoderOverride(uint8_t encoder_index)
{
    int16_t signed_value;
    uint16_t cmd_override_minus, cmd_override_plus;

    switch (encoder_data[encoder_index].function) {
        case (spindle_override):
            cmd_override_minus = CMD_OVERRIDE_SPINDLE_FINE_MINUS;
            cmd_override_plus  = CMD_OVERRIDE_SPINDLE_FINE_PLUS;
            break;

        case (feed_override):
            cmd_override_minus = CMD_OVERRIDE_FEED_FINE_MINUS;
            cmd_override_plus  = CMD_OVERRIDE_FEED_FINE_PLUS;
            break;

        default:
            return;

    }

    signed_value = encoder_data[encoder_index].raw_value - encoder_data[encoder_index].last_raw_value;
    signed_value = signed_value / encoder_data[encoder_index].ticks_per_request;

    // don't do any overrides if not initialised, just store the initial reading
    if (!init_ok) {
        encoder_data[encoder_index].last_raw_value = encoder_data[encoder_index].raw_value;
        return;
    }

    if (signed_value) {

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

    encoder_data[encoder_index].last_raw_value = encoder_data[encoder_index].raw_value;
}

static void processEncoderJog(uint8_t encoder_index, uint8_t jog_axis)
{

    int16_t signed_value;
    char command[30] = "";
    bool jogOkay = (grbl_state == STATE_IDLE || (grbl_state & STATE_JOG));

    signed_value = encoder_data[encoder_index].raw_value - encoder_data[encoder_index].last_raw_value;
    signed_value = signed_value / encoder_data[encoder_index].ticks_per_request;

    // don't jog if not initialised - just store the initial reading (so we can't pick up a big jump on startup)
    // don't jog if in smooth mode - is meant for keypad jogging only (large distances requested, and cancelled on key release)
    if (!init_ok || (jog_mode == jog_mode_smooth)) {
        encoder_data[encoder_index].last_raw_value = encoder_data[encoder_index].raw_value;
        return;
    }

    if (signed_value && jogOkay) {
        strcpy(command, "$J=G91");
        strcat(command, axis[jog_axis]);

        switch (jog_mode) {

            case (jog_mode_x1):
                strcat(command, ftoa(signed_value * JOG_DISTANCE_X1, 3));
                strcat(command, "F");
                strcat(command, ftoa(JOG_SPEED_X1, 0));
                break;

            case (jog_mode_x10):
                strcat(command, ftoa(signed_value * JOG_DISTANCE_X10, 3));
                strcat(command, "F");
                strcat(command, ftoa(JOG_SPEED_X10, 0));
                break;

            case (jog_mode_x100):
                strcat(command, ftoa(signed_value * JOG_DISTANCE_X100, 3));
                strcat(command, "F");
                strcat(command, ftoa(JOG_SPEED_X100, 0));
                break;

            default:
                 break;

        }

        if (!plan_check_full_buffer()) {
            if (grbl.enqueue_gcode((char *)command)) {
                // update last value, only if command was accepted
                encoder_data[encoder_index].last_raw_value = encoder_data[encoder_index].raw_value;
            }
        }
    }
}

static void processEncoders()
{
    for (int i = 0; i < N_ENCODERS; i++) {

        switch (encoder_data[i].function) {
            case (jog_mpg):
                processEncoderJog(i, mpg_axis);
                break;

            case (jog_x):
                processEncoderJog(i, 0);
                break;

            case (jog_y):
                processEncoderJog(i, 1);
                break;

            case (jog_z):
                processEncoderJog(i, 2);
                break;

            case (feed_override):
            case (spindle_override):
                processEncoderOverride(i);
                break;

            default:
                break;

        }

    }
}

static void rx_packet (modbus_message_t *msg)
{
    uint16_t keydata[N_KEYPADS];

    if(!(msg->adu[0] & 0x80)) {

        switch((panel_response_t)msg->context) {

            case Panel_ReadInputRegisters:
                encoder_data[0].raw_value = (msg->adu[7] << 8)  | msg->adu[8];      // Register 102
                encoder_data[1].raw_value = (msg->adu[9] << 8)  | msg->adu[10];     // Register 103

                keydata[0] = (msg->adu[15] << 8) | msg->adu[16];                    // Register 106
                keydata[2] = (msg->adu[19] << 8) | msg->adu[20];                    // Register 108

                processKeypad(keydata);
                processEncoders();

                // after one pass through, have populated the startup encoder values etc..
                init_ok = true;

                break;

            case Panel_WriteHoldingRegisters:
                break;

            default:
                break;
        }
    }

}

static void rx_exception (uint8_t code)
{
    // todo: need a 'Panel' alarm status
    system_raise_alarm(Alarm_None);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:PANEL v0.01]" ASCII_EOL);
    }
}

static void panel_settings_changed (settings_t *settings)
{
    if(settings_changed)
        settings_changed(settings);

    // todo: read settings from nvs, same for the #defines in panel.h

    encoder_data[0].function = jog_mpg;
    encoder_data[0].ticks_per_request = 4;

    encoder_data[1].function = feed_override;
    encoder_data[1].ticks_per_request = 2;
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

    // Initiate Modbus requests to the panel every PANEL_UPDATE_INTERVAL ms
    // alternating inputs (buttons/encoders) and outputs (display)
    if (!(ms % PANEL_UPDATE_INTERVAL) )
    {
        if (!write)
            ReadInputRegisters(false);      // do not block for modbus response
        else
            WriteHoldingRegisters(false);   // do not block for modbus response

        write = !write;
    }

    last_ms = ms;
}

void panel_init()
{
    if(modbus_enabled()) {
        settings_changed = hal.settings_changed;
        hal.settings_changed = panel_settings_changed;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = panel_update;
    }
}
#endif
