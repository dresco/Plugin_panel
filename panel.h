/*

  panel.h - Control panel support

  Part of grblHAL

  Copyright (c) 2020 Terje Io
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

#ifndef _PANEL_H_
#define _PANEL_H_

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#include "spindle/modbus.h"
#include "registers.h"

#ifndef PANEL_UPDATE_INTERVAL
#define PANEL_UPDATE_INTERVAL 50
#endif

#ifndef PANEL_ADDRESS
#define PANEL_ADDRESS 0x0A
#endif

#ifndef PANEL_START_REF
#define PANEL_START_REF 100
#endif

#ifndef PANEL_READREG_COUNT
#define PANEL_READREG_COUNT 16
#endif

#ifndef PANEL_WRITEREG_COUNT
#define PANEL_WRITEREG_COUNT 11
#endif

#define N_KEYPADS  5
#define N_ENCODERS 4

#define JOG_DISTANCE_X1     0.01
#define JOG_DISTANCE_X10    0.1
#define JOG_DISTANCE_X100   1
#define JOG_DISTANCE_SMOOTH 10

#define JOG_SPEED_X1        10
#define JOG_SPEED_X10       100
#define JOG_SPEED_X100      1000
#define JOG_SPEED_SMOOTH    100

typedef enum {
    Panel_Idle = 0,
    Panel_ReadInputRegisters,
    Panel_WriteHoldingRegisters
} panel_response_t;

typedef enum {
    jog_mode_x1 = 1,
    jog_mode_x10 = 2,
    jog_mode_x100 = 4,
    jog_mode_smooth = 8
} panel_jog_mode_t;

typedef enum {
    null             = 0,
    spindle_override = 1,
    feed_override    = 2,
    rapid_override   = 3,
    jog_mpg          = 4,   // typical single encoder for jogging, axis selected seperately
    jog_x            = 5,   // dedicated per-axis encoders
    jog_y            = 6,
    jog_z            = 7,
    jog_a            = 8,
    jog_b            = 9
} panel_encoder_function_t;

typedef struct {
    uint8_t                  ticks_per_request;
    uint16_t                 raw_value;
    uint16_t                 last_raw_value;
    panel_encoder_function_t function;
} panel_encoder_data_t;

void panel_init ();

#endif /* _PANEL_H_ */
