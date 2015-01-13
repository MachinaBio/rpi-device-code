/*
    SE95 - An arduino library for the SE95 temperature sensor
    Copyright (C) 2012  Dan Fekete <thefekete AT gmail DOT com>
    modified from 
    Copyright (C) 2011  Dan Fekete <thefekete AT gmail DOT com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SE95_h

#define SE95_h

#include "Arduino.h"

#define SE95_ADDRESS 0x48

#define SE95_TEMP_REGISTER 0
#define SE95_CONF_REGISTER 1
#define SE95_THYST_REGISTER 2
#define SE95_TOS_REGISTER 3

#define SE95_CONF_SHUTDOWN  0
#define SE95_CONF_OS_COMP_INT 1
#define SE95_CONF_OS_POL 2
#define SE95_CONF_OS_F_QUE 3

class SE95 {
    int address;
    word float2regdata (float);
    float regdata2float (word);
    word _register16 (byte);
    void _register16 (byte, word);
    word _register8 (byte);
    void _register8 (byte, byte);
  public:
    SE95 ();
    SE95 (byte);
    float temp (void);
    byte conf (void);
    void conf (byte);
    float tos (void);
    void tos (float);
    float thyst (void);
    void thyst (float);
    void shutdown (boolean);
    boolean shutdown (void);
};

#endif
