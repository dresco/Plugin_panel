/*

  registers.h - Modbus register definitions for control panel support

  Part of grblHAL

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

typedef union {
    uint16_t value;
    struct {
        uint16_t stop                   :1,
                 feed_hold              :1,
                 cycle_start            :1,
                 reset                  :1,
                 unlock                 :1,
                 home                   :1,
                 unused                 :1,
                 function_f1            :1,
                 function_f2            :1,
                 function_f3            :1,
                 function_f4            :1,
                 mpg_axis_x             :1,
                 mpg_axis_y             :1,
                 mpg_axis_z             :1,
                 mpg_axis_a             :1,
                 mpg_axis_b             :1;
    };
} panel_keydata_1_t;

typedef union {
    uint16_t value;
    struct {
        uint16_t wcs_g54                :1,
                 wcs_g55                :1,
                 wcs_g56                :1,
                 wcs_g57                :1,
                 unused                 :2,
                 zero_work_offset_x     :1,
                 zero_work_offset_y     :1,
                 zero_work_offset_z     :1,
                 zero_work_offset_a     :1,
                 zero_work_offset_b     :1,
                 move_to_zero_x         :1,
                 move_to_zero_y         :1,
                 move_to_zero_z         :1,
                 move_to_zero_a         :1,
                 move_to_zero_b         :1;
    };
} panel_keydata_2_t;

typedef union {
    uint16_t value;
    struct {
        uint16_t jog_negative_x         :1,
                 jog_positive_x         :1,
                 jog_negative_y         :1,
                 jog_positive_y         :1,
                 jog_negative_z         :1,
                 jog_positive_z         :1,
                 jog_negative_a         :1,
                 jog_positive_a         :1,
                 jog_negative_b         :1,
                 jog_positive_b         :1,
                 unused                 :2,
                 jog_step_x1            :1,
                 jog_step_x10           :1,
                 jog_step_x100          :1,
                 jog_step_smooth        :1;
    };
} panel_keydata_3_t;


typedef union {
    uint16_t value;
    struct {
        uint16_t feed_coarse_minus      :1,
                 feed_fine_minus        :1,
                 feed_fine_plus         :1,
                 feed_coarse_plus       :1,
                 feed_override_reset    :1,
                 spindle_coarse_minus   :1,
                 spindle_fine_minus     :1,
                 spindle_fine_plus      :1,
                 spindle_coarse_plus    :1,
                 spindle_override_reset :1,
                 rapid_override_25      :1,
                 rapid_override_50      :1,
                 rapid_override_100     :1,
                 unused                 :3;
    };
} panel_keydata_4_t;


typedef union {
    uint16_t value;
    struct {
        uint16_t unused                 :16;
    };
} panel_keydata_5_t;


