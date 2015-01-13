/*
 Kernel Controller  
 Biological Incubation // Controller
 
 Releases
 v1 - July 16, 2012
 
 Authors
 Carlo Quinonez
 Sidra Haiy
 
 This software is Copyright © 2011 The Regents of the University of California. All Rights
 Reserved.
 
 Permission to use, copy, modify, and distribute these design files, software, firmware and
 documentation for educational, research and non-profit purposes, without fee, and without
 a written agreement is hereby granted, provided that the above copyright notice, this
 paragraph and the following three paragraphs appear in all copies.
 
 Permission to make commercial use of this software may be obtained by contacting:
 Technology Transfer Office
 9500 Gilman Drive, Mail Code 0910
 University of California
 La Jolla, CA 92093-0910
 (858) 534-5815
 invent@ucsd.edu
 
 These design files, software, firmware and documentation are copyrighted by The Regents
 of the University of California. The software program and documentation are supplied "as is",
 without any accompanying services from The Regents. The Regents does not warrant that the
 operation of the program will be uninterrupted or error-free. The end-user understands that
 the program was developed for research purposes and is advised not to rely exclusively on
 the program for any reason.
 
 IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
 SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
 USE OF THESE DESIGN FILES, SOFTWARE, FIRMWARE AND DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 CALIFORNIA HAS BEEN ADVISED OFTHE POSSIBILITY OF SUCH DAMAGE. THE UNIVERSITY OF CALIFORNIA
 SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATIONS TO PROVIDE
 MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS. 
 
 */

///////////////////////////////////////////////////
// VARIABLE DECLARATIONS are made prior to loading
// any external libraries, thus no external dependencies
///////////////////////////////////////////////////

// Variables for PID control algorithm. Type is set by PID library.
double tempTopActual, tempSubActual, humidityActual, tempWater, tempAir;  // in degress Celcius
double heaterTop, heaterSub, heaterWater;  // from 0-355.
double tempTopSet, tempSubSet, humiditySet, tempHumSet;  // in degrees Celcius

int portA, portB; // 0=ok, 1=not connected, 2=i2c error, 4=wrong device
bool recheckA, recheckB; //
unsigned long startTime, nextTime;

///////////////////////////////////////////////////
// Externals libraries and pin assignments
//  are loaded after creating local dependancies (ie. pointers).
///////////////////////////////////////////////////
#include <Wire.h>
#include <math.h>
#include <inttypes.h>
#include <PCA9540.h>
#include <LCDi2cNHD.h>
#include <SE95.h>
#include <SensirionI2C.h>
#include <LTC1695.h>
#include <PID.h>
#include <Scheduler.h>
#include <TimedAction.h>
#include "kernelController.h"

///////////////////////////////////////////////////
// Create control objects for hardware.
///////////////////////////////////////////////////
LCDi2cNHD lcd = LCDi2cNHD(4,20,0x50>>1,0);  // 2 lines x 16 characters
PCA9540 i2cBus = PCA9540();
SE95 sensor0x48 = SE95(0x48);
SE95 sensor0x49 = SE95(0x49);
LTC1695 fan = LTC1695();
SensirionI2C sht75 = SensirionI2C(dataPin, clockPin); 

PID stagetopAlgorithm = PID(&tempAir, &heaterTop, &tempTopSet, 
aggKp, aggKi, aggKd, DIRECT);
PID humidityAlgorithm = PID(&humidityActual, &heaterWater, &humiditySet,
humKp, humKi, humKd, DIRECT);
PID substageAlgorithm = PID(&tempSubActual, &heaterSub, &tempSubSet,
ssKp, ssKi, ssKd, DIRECT);
/*                           
 TimedAction checkSensirionsAction = TimedAction(3000, checkSensirions);
 TimedAction checkSE95sAction = TimedAction(500,  checkSE95s);
 TimedAction redrawScreen = TimedAction(
 */

#include "fscale.h"
#include "utilities.h"

///////////////////////////////////////////////////
// One time setup
///////////////////////////////////////////////////
void setup() {  
  setupPins();

   setupSerial(); // turn on hardware echo on the serial port. 
   
   //  Serial.begin(9600);
 // Serial.print("Time(s), Air temp, Chamber temp, Substage temp, Water temp, RH %, Substage htr power, chamber htr power, water htr power");


  lcd.init(); // also calls Wire.begin

  TWBR=152; /* == 50kHz SCL frequency */
  lcd.setDelay(5,0);

  // set the setpoints
  tempTopSet=37;  // in C
  tempSubSet=35;  // in C
  tempHumSet=34;  // in C
  humiditySet=95; // in % RH

  // make sure all the heaters are off initially
  heaterTop   = 0;
  heaterSub   = 0;
  heaterWater = 0;

  // basic PID setup
  stagetopAlgorithm.SetOutputLimits(-45, 49); 
  stagetopAlgorithm.SetSampleTime(3000);
  stagetopAlgorithm.SetMode(AUTOMATIC);

  substageAlgorithm.SetOutputLimits(-50, 49); 
  substageAlgorithm.SetSampleTime(3000);
  substageAlgorithm.SetMode(AUTOMATIC);

  humidityAlgorithm.SetOutputLimits(-50, 49);
  humidityAlgorithm.SetSampleTime(3000);
  humidityAlgorithm.SetMode(AUTOMATIC);

  // turn on the fans
  i2cBus.enableChannel0();
  fan.fullSpeed();
  sht75.reset();
  
  i2cBus.enableChannel1();
  fan.fullSpeed();
  sht75.reset();

  // show the splash screen
  splashScreen();
 
  startTime = millis();
  nextTime = millis()+1000;
}

///////////////////////////////////////////////////
// Processed continuously.
///////////////////////////////////////////////////
void loop() {
  // Variables for Sensirion SH75 temperature and humidity sensor. Type is set by the Sensirion library.
  unsigned long timer;
  float temperature, humidity, dewpoint;
  int pwmPercent;
  int scaledTopPower, scaledHumPower, scaledSubPower;

  timer = millis();
  
  /////////////////////////////
  /////////////////////////////  
  // MEASURE STAGE TOP 
  /////////////////////////////
  /////////////////////////////
  i2cBus.enableChannel1(); // ENABLE PORT A
  if ( i2cBus.whichChannel() == 1 ) {
    fan.fullSpeed();
    sht75.reset();
    delay(15);
    ///////////// AIR TEMP
    // read sht75 sensor and process the data
    temperature = 0;
    humidity = 0;
    dewpoint = 0;
    sht75.measure(&temperature, &humidity, &dewpoint);

    tempAir = temperature;
    humidityActual = humidity;

    ///////////// HUMIDITY
    // Read the water temperature
    tempWater = sensor0x48.temp(); // Copy valvue into the humidity PID
    // Adjust the water heater based on the temp
    humidityAlgorithm.Compute();

    ///////////// THERMAL TUB
    // Read the temperature of the culture chamber housing
    tempTopActual = sensor0x49.temp();
    // Adjust the chamber heater based on the chamber temperature
    stagetopAlgorithm.Compute();
  }
  else {
    lcd.clear();
    lcd.print("Channel1 error");
    delay(1000);
  }

  /////////////////////////////
  // MEASURE SUBSTAGE 
  /////////////////////////////

  i2cBus.enableChannel0(); // ENABLE PORT B
  if ( i2cBus.whichChannel() == 0) { 
    fan.fullSpeed();
        sht75.reset();
    delay(15);

    temperature = 0;
    humidity = 0;
    dewpoint = 0;

    ///////////// SUBSTAGE
    // read sht75 sensor and process the data
    sht75.measure(&temperature, &humidity, &dewpoint);

    // Adjust the water heater based on the relative humidity
    tempSubActual = temperature; // Copy value into the humidity PID
    substageAlgorithm.Compute();
  }
  else {
    lcd.clear();
    lcd.print("Channel0 error");
    delay(1000);
  }
  
  if (tempWater < 5) heaterWater = NOPOWER; // UNDER TEMP - restry. Probably an unconnected port.
  else if (tempWater > 50) {  // OVER TEMP - latched fault. Must reset controller.
    lcd.clear();
    lcd.print("Overtemp Water");
    lcd.setCursor(1,0);
    lcd.print("Reset controller");
    latchFault();
  }

  if (tempTopActual < 5) heaterTop = NOPOWER;
  else if (tempTopActual > 45 ) { // OVER TEMP - latched fault. Must reset controller.
    lcd.clear();
    lcd.print("Overtemp Culture");
    lcd.setCursor(1,0);
    lcd.print("Reset controller");
    latchFault();
  }

  if (tempSubActual < 5) heaterSub = NOPOWER;
  else if (tempSubActual > 45 ) { // OVER TEMP - latched fault. Must reset controller.
    lcd.clear();
    lcd.print("Overtemp Substage");
    lcd.setCursor(1,0);
    lcd.print("Reset controller");
    latchFault();
  }

  checkPorts();

  if (portA){ // error with portA - turn off the heaters!
    heaterTop = NOPOWER;
    heaterWater=NOPOWER;
  }

  if (portB) heaterSub= NOPOWER; // error with portB - turn off the heaters!
  int rescaled;

  scaledHumPower = fscale(-50.0, 49.0, 0.0, HUMIDITY_HEATER_MAX, heaterWater, 0);  
  scaledTopPower = fscale(-50.0, 49.0, 0.0, HUMIDITY_CULTURE_MAX, heaterTop, 0);
  scaledSubPower = fscale(-50.0, 49.0, 0.0, HUMIDITY_SUBSTAGE_MAX, heaterSub, 0);

  analogWrite(heaterB2, scaledSubPower);
  analogWrite(heaterA1, scaledTopPower);
  analogWrite(heaterA2, scaledHumPower);
  
  drawScreen(); //  clear and redraw the screen
  drawValues();

  // print ram
  //  lcd.setCursor(3,16);
  //  lcd.print(freeRam());

//  Serial.print("\n");
//  Serial.print( (millis()-startTime) / 1000 );
//  Serial.print(",");
//  Serial.print(tempAir,2);
//  Serial.print(",");
//  Serial.print(tempTopActual,2);
//  Serial.print(",");
//  Serial.print(tempSubActual,2);
//  Serial.print(",");
//  Serial.print(tempWater,2);
//  Serial.print(",");
//  Serial.print(humidityActual,1);
//  Serial.print("%,");
//  Serial.print(scaledSubPower,1);
//  Serial.print(",");
//  Serial.print(scaledTopPower,1);
//  Serial.print(",");
//  Serial.print(scaledHumPower,1);

  if (nextTime < millis()) { // time to exchange the airtemp
    if (humidityActual < 85) {
      digitalWrite(valve1, HIGH);
      digitalWrite(valve2, HIGH);
      nextTime += 12000;
          lcd.setCursor(1,17);
    lcd.print("2x");
    }
    else {
      digitalWrite(valve1, HIGH);
      nextTime += 300000;
      lcd.setCursor(1,17);
      lcd.print("1x");
    }
  }
  else {
    digitalWrite(valve1, LOW);
    digitalWrite(valve2, LOW);
    lcd.setCursor(1,17);
    lcd.print("Off");
  }
  delay(3000);
}







