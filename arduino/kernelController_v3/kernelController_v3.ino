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
double tempTopActual, tempSubActual, humidityActual, tempWater, tempAir, tempSubstageBoard;  // in degress Celcius
double heaterTop, heaterSub, heaterWater;  // from 0-355.
float chamberHtrAmps, waterHtrAmps, htrSupplyVolts, htrB1Amps, htrB2Amps;

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

PID stagetopAlgorithm = PID(&tempTopActual, &heaterTop, &tempTopSet, consKp, consKi, consKd, DIRECT);
//PID humidityAlgorithm = PID(&humidityActual, &heaterWater, &humiditySet, humKp, humKi, humKd, DIRECT);
PID humidityAlgorithm = PID(&tempWater, &heaterWater, &tempHumSet, humKp, humKi, humKd, DIRECT);
PID substageAlgorithm = PID(&tempSubActual, &heaterSub, &tempSubSet, ssKp, ssKi, ssKd, DIRECT);

////////////////// DATA LOG / DEBG SWITCH
////////////////// DATA LOG / DEBG SWITCH
// Define LOGDATA to add serial debug log. Unfortunately, also disables intrastack network interface.
// #define LOGDATA 1
////////////////// DATA LOG / DEBG SWITCH
////////////////// DATA LOG / DEBG SWITCH

#include "fscale.h"
#include "utilities.h"
///////////////////////////////////////////////////
// One time setup
///////////////////////////////////////////////////
void setup() {
  lcd.init(); // also calls Wire.begin
  lcd.setDelay(5,0);
  TWBR=152; /* == 50kHz SCL frequency */
  
  splashScreen();
  
#if LOGDATA    
  Serial.begin(9600);
  Serial.println("Incubator - BT Stack Datalogger - v1");
  Serial.println("Time(s),Air temp,RH %,Chamber temp,Objective temp,Humidifier temp,SubstageBrd temp,Substage PWM,chamber PWM,humidifier PWM,chamber A,humidifier A,HeaterB1 A,HeaterB2 A,48V rail V,Vcc rail mV,free ram Bytes,loopTime (ms)");
#else
  setupSerial(); // turn on hardware echo on the serial port. 
#endif
  
  setupPins();

  // make sure all the heaters are off initially
  heaterTop   = 0;
  heaterSub   = 0;
  heaterWater = 0;

  // basic PID setup
  stagetopAlgorithm.SetOutputLimits(-500, 499); 
  stagetopAlgorithm.SetSampleTime(4000);
  stagetopAlgorithm.SetMode(AUTOMATIC);

  substageAlgorithm.SetOutputLimits(-500, 499); 
  substageAlgorithm.SetSampleTime(4000);
  substageAlgorithm.SetMode(AUTOMATIC);

  humidityAlgorithm.SetOutputLimits(-500, 499);
  humidityAlgorithm.SetSampleTime(4000);
  humidityAlgorithm.SetMode(AUTOMATIC);

  startTime = millis();
}

///////////////////////////////////////////////////
// Processed continuously.
///////////////////////////////////////////////////
void loop() {
  // Variables for Sensirion SH75 temperature and humidity sensor. Type is set by the Sensirion library.
  unsigned long loopStart;
  float temperature, humidity, dewpoint;
  int pwmPercent;
  int scaledTopPower, scaledHumPower, scaledSubPower;

  unsigned long loopstart = millis();
  nextTime = loopstart+4000;

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
    humidityAlgorithm.Compute();   // Adjust the water heater based on the temp

    ///////////// THERMAL TUB
    tempTopActual = sensor0x49.temp();    // Read the temperature of the culture chamber housing
    stagetopAlgorithm.Compute();          // Adjust the chamber heater based on the chamber temperature
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

    ///////////// AIR TEMP AT OBJECTIVES
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
  
    ///////////// BOARD TEMP AT SUBSTAGE BOARD   
    tempSubstageBoard = sensor0x48.temp(); 
  
  ///////////// LIMIT CHECKING AND ERROR HANDLING

  if (tempWater < 10) heaterWater = NOPOWER; // UNDER TEMP - restry. Probably an unconnected port.
  else if (tempWater > 45) {  // OVER TEMP - latched fault. Must reset controller.
    lcd.clear();
    lcd.print("Overtemp Water");
    lcd.setCursor(1,0);
    lcd.print("Reset controller");
    latchFault();
  }

  if (tempTopActual < 10) heaterTop = NOPOWER;
  else if (tempTopActual > 45 ) { // OVER TEMP - latched fault. Must reset controller.
    lcd.clear();
    lcd.print("Overtemp Culture");
    lcd.setCursor(1,0);
    lcd.print("Reset controller");
    latchFault();
  }

  if (tempSubActual < 10) heaterSub = NOPOWER;  
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

  scaledHumPower = fscale(-500.0, 499.0, 0.0, HUMIDITY_HEATER_MAX, heaterWater, 0);  
  scaledTopPower = fscale(-500.0, 499.0, 0.0, HUMIDITY_CULTURE_MAX, heaterTop, 0);
  scaledSubPower = fscale(-500.0, 499.0, 0.0, HUMIDITY_SUBSTAGE_MAX, heaterSub, 0);

  analogWrite(heaterB2, scaledSubPower);
  analogWrite(heaterA2, scaledHumPower);
  analogWrite(heaterA1, scaledTopPower);

  //////////////////////////////
  // Analog INPUTS

  // Read water heater power
  analogReference(DEFAULT); // use the 5V internal reference. Fullscale current 1.2 A @ 10k Ohm feedback resistor  
  waterHtrAmps = readCurrentCounts(heaterA2fb);
  if (waterHtrAmps < 210) // use the lower range
  {
    analogReference(INTERNAL); // use the 1.1V internal reference. Fullscale current 0.264 A @ 10k Ohm feedback resistor    
    waterHtrAmps = readCurrentCounts(heaterA2fb);
  }

  // Read culture heater power
  analogReference(DEFAULT); // use the 5V internal reference. Fullscale current 1.2 A @ 10k Ohm feedback resistor  
  chamberHtrAmps = readCurrentCounts(heaterA1fb);
  if (chamberHtrAmps < 210) // use the lower range
  {
    analogReference(INTERNAL); // use the 1.1V internal reference. Fullscale current 0.264 A @ 10k Ohm feedback resistor    
    chamberHtrAmps = readCurrentCounts(heaterA1fb);
  }
  
    // Read Heater B1 current
  analogReference(DEFAULT); // use the 5V internal reference. Fullscale current 1.2 A @ 10k Ohm feedback resistor  
  htrB1Amps = readCurrentCounts(heaterB1fb);
  if (htrB1Amps < 210) // use the lower range
  {
    analogReference(INTERNAL); // use the 1.1V internal reference. Fullscale current 0.264 A @ 10k Ohm feedback resistor    
    htrB1Amps = readCurrentCounts(heaterB1fb);
  }
  
    // Read Heater B2 current
  analogReference(DEFAULT); // use the 5V internal reference. Fullscale current 1.2 A @ 10k Ohm feedback resistor  
  htrB2Amps = readCurrentCounts(heaterB2fb);
  if (htrB2Amps < 210) // use the lower range
  {
    analogReference(INTERNAL); // use the 1.1V internal reference. Fullscale current 0.264 A @ 10k Ohm feedback resistor    
    htrB2Amps = readCurrentCounts(heaterB2fb);
  }

  // Read 48V supply rail
  analogReference(DEFAULT); // use the 5V internal reference. Fullscale voltage 55V @ 100k/10k resistor divider  
  htrSupplyVolts = readCurrentCounts(statusVcc);

  drawScreen(); //  clear and redraw the screen
  drawValues();

  // Log serial following information:
  // Time(s), Air temp, Chamber temp, Substage temp, Water temp, 
  // RH %, Substage htr PWM, chamber htr PWM, water htr PWM, 
  // chamber hrt Amps, water hrt Amps, Heater rail Volts, free ram Bytes

#if LOGDATA
  Serial.print("\n");
  Serial.print( (float) (millis()-startTime) / 1000.0, 2 );
  Serial.print(",");
  Serial.print(tempAir,2);
  Serial.print(",");
  Serial.print(humidityActual,1);
  Serial.print("%,");
  Serial.print(tempTopActual,2);
  Serial.print(",");
  Serial.print(tempSubActual,2);
  Serial.print(",");
  Serial.print(tempWater,2);
  Serial.print(",");
  Serial.print(tempSubstageBoard,2);
  Serial.print(",");
  Serial.print(scaledSubPower);
  Serial.print(",");
  Serial.print(scaledTopPower);
  Serial.print(",");
  Serial.print(scaledHumPower);
  Serial.print(",");
  Serial.print(chamberHtrAmps,3);
  Serial.print(",");
  Serial.print(waterHtrAmps,3);
  Serial.print(",");
  Serial.print(htrB1Amps,3);
  Serial.print(",");
  Serial.print(htrB2Amps,3);
  Serial.print(",");  
  Serial.print(htrSupplyVolts,3);
  Serial.print(",");
  Serial.print(readVcc(),3);  
  Serial.print(",");
  Serial.print(freeRam());
  Serial.print(",");
  Serial.print(millis()-loopstart);
#endif

  lcd.setCursor(1,17);

  if (humidityActual < (humiditySet - 5)) {
    digitalWrite(valve1, HIGH);
    digitalWrite(valve2, HIGH);
    lcd.print("3x");
  }
  else if (humidityActual < (humiditySet - 1)) {
    digitalWrite(valve1, LOW); // low flow
    digitalWrite(valve2, HIGH); // high flow
    lcd.print("2x");
  }
  else if (humidityActual < (humiditySet - .5)) {
    digitalWrite(valve1, HIGH); // low flow
    digitalWrite(valve2, LOW); // high flow
    lcd.print("1x");
  }
  else {
    digitalWrite(valve1, LOW); // low flow
    digitalWrite(valve2, LOW); // high flow
    lcd.print("off");
  }

  while ( nextTime > millis() ) {
  }; // don't exit this loop until 3000 ms have elapsed since the start
}








