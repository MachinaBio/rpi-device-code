/* ================================================================== */
/*	DS28CM00.h - Library for Maxim DS28CM00 I2C silicon serial number */
/*	Created by Carlo J Quinonez, July 11, 2012	      	              */
/* ================================================================== */

#ifndef DS28CM00_h
#define DS28CM00_h
#include <Arduino.h>  // Only compatible with Arduino 100+

const byte I2C_MODE = TRUE 
// TRUE = turn off the bus timeout feature (I2C mode)
// FALSE = enable the timeout (SMBUS mode)

 class DS28CM00 {
 long long serial;
 
 private:
   bool validCRC ( byte CRC );
 
 public:
   DS28CM00(void);
   byte serial(void);
   byte reset(void);
};
 
 #endif  // #ifndef DS28CM00_h