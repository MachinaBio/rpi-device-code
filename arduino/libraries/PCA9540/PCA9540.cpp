/* ======================================================= */
/*  PCA9540.cpp - Library for NXP PCA9540 I2C multiplexer  */
/*  Created by Carlo J Quinonez, July 9, 2012              */
/* ======================================================= */

#include "PCA9540.h"
#include <Wire.h>

/********************************************************
 * Definitions
 ********************************************************/
// PCA9540 command register settings
const uint8_t ENABLE_0 = B00000100;
const uint8_t ENABLE_1 = B00000101;  
const uint8_t DISABLE  = B00000000;

// PCA9540 status register masks and values
const uint8_t MASK   = B00000111;
const uint8_t CHAN0  = B00000100;
const uint8_t CHAN1  = B00000101;

const uint8_t PCA9540_ADDRESS = 0x70;  // 7-bit address


/********************************************************
 * Constructors
 ********************************************************/
/* Useless constructor since there's only 1 available address
PCA9540::PCA9540(byte addr) {
  address = addr;
} */

PCA9540::PCA9540 () {
  address = PCA9540_ADDRESS;
}

/********************************************************
 * Public functions
 ********************************************************/
 void PCA9540::enableChannel0() {
  Wire.beginTransmission(address);
  Wire.write(ENABLE_0);
  Wire.endTransmission();
 }
 
 void PCA9540::enableChannel1()  {
  Wire.beginTransmission(address);
  Wire.write(ENABLE_1);
  Wire.endTransmission();
 }
 
 void PCA9540::disable() {
  Wire.beginTransmission(address);
  Wire.write(DISABLE);
  Wire.endTransmission();
 }

  int PCA9540::whichChannel() {
  uint8_t statusReg;
    
  Wire.requestFrom(address, (uint8_t) 1);
  statusReg = Wire.read();
  statusReg &= MASK;
  
  if ( statusReg == CHAN0 ) return 0;
  if ( statusReg == CHAN1 ) return 1;
  return -1;
 }
 
/********************************************************
 * PRIVATE Communication functions
 ********************************************************/


/********************************************************
 * PRIVATE Helper functions
 ********************************************************/
