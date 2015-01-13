/* ============================================================= */
/*	LTC1695.cpp - Library for Linear LTC1695 I2C fan controller  */
/*	Created by Carlo J Quinonez, July 9, 2012	      	         */
/* ============================================================= */

#include "LTC1695.h"
#include <Wire.h>

/********************************************************
 * Definitions
 ********************************************************/
const byte LTC1695_ADDRESS = 0x74;
const byte FULL_SPEED = 63;
const byte HALF_SPEED = 31;
const byte BOOST_START = B01000000;

// LTC1695 register masks
const byte OVERTEMP_FAULT     = 0x40;	 
const byte OVERCURRENT_FAULT  = 0x80;

/********************************************************
 * Constructors
 ********************************************************/
LTC1695::LTC1695() {
  currentSpeed = 0;
  }

/********************************************************
 * Public functions
 ********************************************************/
byte LTC1695::fullSpeed() {
  return adjustSpeed(FULL_SPEED);
  }
 
byte LTC1695::halfSpeed()	{
  return adjustSpeed(HALF_SPEED);
  }
 
 
byte LTC1695::stop() {
  return adjustSpeed(0);
  }

byte LTC1695::adjustSpeed(byte newSpeed) {
  if (newSpeed > 63) newSpeed=63;
  if (currentSpeed == 0) newSpeed |= BOOST_START;
  currentSpeed = newSpeed;
  Wire.beginTransmission(LTC1695_ADDRESS);
  Wire.write(newSpeed);
  return Wire.endTransmission();
  }

bool LTC1695::currentFault() {
  return (statusRegister() == OVERCURRENT_FAULT);
  }

bool LTC1695::tempFault() {
  return (statusRegister() == OVERTEMP_FAULT);
  }

byte LTC1695::getSpeed() {
    return currentSpeed;
    }

/********************************************************
 * PRIVATE Communication functions
 ********************************************************/
byte LTC1695::statusRegister() {
  Wire.requestFrom(LTC1695_ADDRESS, (byte) 1);
  return Wire.read();
}

/********************************************************
 * PRIVATE Helper functions
 ********************************************************/
