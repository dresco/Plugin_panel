/*

  panel.h - Control panel support

  Part of grblHAL

  Copyright (c) 2021-2024 Jon Escombe

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _PANEL_H_
#define _PANEL_H_

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if PANEL_ENABLE == 1 || PANEL_ENABLE == 2

#include <stdio.h>

#if GRBL_BUILD >= 20230610
#include "spindle/modbus_rtu.h"
#else
#include "spindle/modbus.h"
#endif

#if VFD_ENABLE
#include "spindle/vfd/spindle.h"
#endif

#include "keypad_bitfields.h"
#include "canbus_ids.h"

#define N_KEYDATAS 6
#define N_ENCODERS 4

#define PANEL_DEFAULT_UPDATE_INTERVAL     50         // Default update interval (ms)
#define PANEL_DEFAULT_MODBUS_ADDRESS      0x0A       // Default modbus address
#define PANEL_DEFAULT_SPINDLE_SPEED       1000       // Default spindle speed for cw/ccw buttons

#define PANEL_DEFAULT_JOG_DISTANCE_X1     0.01
#define PANEL_DEFAULT_JOG_DISTANCE_X10    0.1
#define PANEL_DEFAULT_JOG_DISTANCE_X100   1
#define PANEL_DEFAULT_JOG_DISTANCE_KEYPAD 4

#define PANEL_DEFAULT_JOG_SPEED_X1        10
#define PANEL_DEFAULT_JOG_SPEED_X10       100
#define PANEL_DEFAULT_JOG_SPEED_X100      1000
#define PANEL_DEFAULT_JOG_SPEED_KEYPAD    2000

#define PANEL_DEFAULT_JOG_KEYPAD_RAMP     20

#ifndef PANEL_MODBUS_START_REG
#define PANEL_MODBUS_START_REG 100
#endif

#ifndef PANEL_MODBUS_READREG_COUNT
#define PANEL_MODBUS_READREG_COUNT 16
#endif

#ifndef PANEL_MODBUS_WRITEREG_COUNT
#define PANEL_MODBUS_WRITEREG_COUNT 13
#endif

typedef enum {
    Panel_Idle = 0,
    Panel_ReadInputRegisters,
    Panel_WriteHoldingRegisters
} panel_modbus_response_t;

typedef enum {
    jog_mode_x1 = 1,
    jog_mode_x10 = 2,
    jog_mode_x100 = 4,
    jog_mode_smooth = 8
} panel_jog_mode_t;

typedef enum {
    unused           = 0,
    spindle_override = 1,
    feed_override    = 2,
    rapid_override   = 3,
    jog_mpg          = 4,   // typical single encoder for jogging, axis selected at run time
    jog_x            = 5,   // dedicated per-axis encoders
    jog_y            = 6,
    jog_z            = 7,
    jog_a            = 8,
    jog_b            = 9,
    jog_c            = 10,
    jog_u            = 11,
    jog_v            = 12
} panel_encoder_mode_t;

typedef struct {
    uint8_t              init_ok;
    uint8_t              cpd;
    uint16_t             raw_value;
    uint16_t             last_raw_value;
    panel_encoder_mode_t mode;
} panel_encoder_data_t;

typedef union {
    float   value;
    uint8_t bytes[4];
} float32_data_t;

typedef struct {
    uint16_t       grbl_state;
    uint16_t       spindle_speed;
    uint16_t       spindle_load;
    uint8_t        spindle_override;
    uint8_t        feed_override;
    uint8_t        rapid_override;
    uint8_t        wcs;
    uint8_t        mpg_mode;
    uint8_t        jog_mode;
    float32_data_t position[N_AXIS];
} panel_displaydata_t;

typedef struct {
    uint8_t  modbus_address;
    uint16_t update_interval;
    uint16_t spindle_speed;

    uint16_t jog_speed_x1;
    uint16_t jog_speed_x10;
    uint16_t jog_speed_x100;
    uint16_t jog_speed_keypad;

    float    jog_distance_x1;
    float    jog_distance_x10;
    float    jog_distance_x100;
    float    jog_distance_keypad;

    uint8_t  jog_accel_ramp;

    uint8_t  encoder_mode[N_ENCODERS];
    uint8_t  encoder_cpd[N_ENCODERS];
} panel_settings_t;

#endif /* PANEL_ENABLE == 1 || PANEL_ENABLE == 2 */

#endif /* _PANEL_H_ */
