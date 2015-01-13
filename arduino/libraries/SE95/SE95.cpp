/*
    SE95 - An arduino library for the SE95 temperature sensor
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

#include <Wire.h>
#include "SE95.h"

#include <Arduino.h>  // only compatible with Ardiuno 100+

SE95::SE95 () {
  address = SE95_ADDRESS;
}

SE95::SE95 (byte addr) {
  address = addr;
}

word SE95::float2regdata (float temp)
{
  // First multiply by 8 and coerce to integer to get +/- whole numbers
  // Then coerce to word and bitshift 5 to fill out MSB
  // return (word)((int)(temp * 8) << 5);  // for 11-bit data
  return (word)((int)(temp * 32) << 3);  // for 13-bit data
}

float SE95::regdata2float (word regdata)
{
//  return ((float)(int)regdata >> 5) / 8; // for 11-bit data
  return ( (float)((int)regdata >> 3) / 32.0 ); // for 13-bit data
}

word SE95::_register16 (byte reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);	
  Wire.endTransmission();
  
  Wire.requestFrom(address, 2);
  word regdata = (Wire.read() << 8) | Wire.read();
  return regdata;
}

void SE95::_register16 (byte reg, word regdata) {
  byte msb = (byte)(regdata >> 8);
  byte lsb = (byte)(regdata);
  
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(msb);
  Wire.write(lsb);
  Wire.endTransmission();
}

word SE95::_register8 (byte reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);	
  Wire.endTransmission();
  
  Wire.requestFrom(address, 1);
  return Wire.read();
}

void SE95::_register8 (byte reg, byte regdata) {  
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(regdata);
  Wire.endTransmission();
}

float SE95::temp (void) {
  return regdata2float(_register16(SE95_TEMP_REGISTER));
}

byte SE95::conf () {
  return _register8(SE95_CONF_REGISTER);
}

void SE95::conf (byte data) {
  _register8(SE95_CONF_REGISTER, data);
}

float SE95::tos () {
  return regdata2float(_register16(SE95_TOS_REGISTER));
}

void SE95::tos (float temp) {
  _register16(SE95_TOS_REGISTER, float2regdata(temp));
}

float SE95::thyst () {
  return regdata2float(_register16(SE95_THYST_REGISTER));
}

void SE95::thyst (float temp) {
  _register16(SE95_THYST_REGISTER, float2regdata(temp));
}

boolean SE95::shutdown () {
  return conf() & 0x01;
}

void SE95::shutdown (boolean val) {
  conf(val << SE95_CONF_SHUTDOWN);
}
