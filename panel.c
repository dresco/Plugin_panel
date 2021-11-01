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

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

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

uint16_t grbl_state;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};


static void ReadInputRegisters(bool block)
{
}

static void WriteHoldingRegisters(bool block)
{
}


static void rx_packet (modbus_message_t *msg)
{
}

static void rx_exception (uint8_t code)
{
	// TODO: need a 'Panel' alarm status
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
}

void panel_update (sys_state_t grbl_state)
{
    static uint32_t last_ms;
    static bool write = false;

    on_execute_realtime(grbl_state);

    uint32_t ms = hal.get_elapsed_ticks();

    if(ms == last_ms) // check once every ms
        return;

    // Send requests to panel every PANEL_UPDATE_INTERVAL ms, alternating inputs and outputs
    if (!(ms % PANEL_UPDATE_INTERVAL) )
    {
    	if (!write)
    		ReadInputRegisters(false);
    	else
    		WriteHoldingRegisters(false);

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
