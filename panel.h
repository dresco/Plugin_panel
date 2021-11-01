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

#ifndef PANEL_UPDATE_INTERVAL
#define PANEL_UPDATE_INTERVAL 5
#endif

void panel_init ();

#endif /* _PANEL_H_ */
