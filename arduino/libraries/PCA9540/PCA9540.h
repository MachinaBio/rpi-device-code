/* ======================================================= */
/*  PCA9540.h - Library for NXP PCA9540 I2C multiplexer    */
/*  Created by Carlo J Quinonez, July 9, 2012              */
/* ======================================================= */

#ifndef PCA9540_h
#define PCA9540_h
#include <Arduino.h>  // Only compatible with Arduino 100+
 
 class PCA9540 {
 public:
 // PCA9540(byte); USELESS - Only 1 available address
 PCA9540();
 void enableChannel0(void);
 void enableChannel1(void);
 void disable(void);
 int whichChannel(void);
 
 private:
 uint8_t address;
 };
 
 #endif  // #ifndef PCA9540_h