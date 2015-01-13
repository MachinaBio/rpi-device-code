/* =========================================================== */
/*  LTC1695.h - Library for Linear LTC1695 I2C fan controller  */
/*  Created by Carlo J Quinonez, July 11, 2012                 */
/* =========================================================== */

#ifndef LTC1695_h
#define LTC1695_h
#include <Arduino.h>  // Only compatible with Arduino 100+
 
 class LTC1695 {
 public:
 LTC1695(void);
 byte fullSpeed(void);
 byte halfSpeed(void);
 byte adjustSpeed(byte); // from 0-63
 byte stop(void);
 bool currentFault(void);
 bool tempFault(void);
 byte getSpeed(void);
 
 private:
 byte currentSpeed;
 byte statusRegister();
 };
 
 #endif  // #ifndef LTC1695_h