/*
bio Controller
 
 Biological Incubation // Controller
 Maintains heated environmental chamber to 37 C (fixed)
 
 Releases
 v1 - April 10 2011
 
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

// Variables for Sensirion SH75 temperature and humidity sensor. Type is set by the Sensirion library.
float temperature, humidity, dewpoint;

// Variables for PID control algorithm. Type is set by PID library.
double measuredTemp;  // in degress Celcius
double heaterPower;  // from 0-355.
double setpoint = 37;  // in degrees Celcius

// Misc globals
unsigned long startTime;

///////////////////////////////////////////////////
// Externals libraries and pin assignments
//  are loaded after creating local dependancies (ie. pointers).
///////////////////////////////////////////////////
#include <Wire.h>
#include <inttypes.h>
#include <LCDi2cNHD.h>
#include <Sensirion.h>
#include <PID.h>

#include "bioController.h"

///////////////////////////////////////////////////
// Create control objects for hardware.
///////////////////////////////////////////////////
LCDi2cNHD lcd = LCDi2cNHD(2,16,0x50>>1,0);  // 2 lines x 16 characters
Sensirion sensor = Sensirion(dataPin, clockPin); 
PID heatingAlgorithm = PID(&measuredTemp, &heaterPower, &setpoint, aggKp, aggKi, aggKd, DIRECT);  // P = 100, I = 2, D = 0

///////////////////////////////////////////////////
// One time setup
///////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);

  digitalWrite(stageDetectPin, LOW);   // set the LED on

  heaterPower = 0 ; // for EC Sample v1

 digitalWrite(stageDetectPin, LOW);

  pinMode(heaterPin, OUTPUT);  
  pinMode(stageDetectPin, INPUT);

  lcd.init();
  lcd.setCursor(1,0);  // hide cursor
  lcd.blink_off(); 

  startTime = millis();  // store start time in milliseconds

  heatingAlgorithm.SetOutputLimits(-15, 255); 
  heatingAlgorithm.SetSampleTime(3000);
  heatingAlgorithm.SetMode(AUTOMATIC);
}

///////////////////////////////////////////////////
// Processed continuously.
///////////////////////////////////////////////////
void loop() {
  double elapsedSeconds;
  float pwmPercent;
  
  // Check that heater is connected
//  if ( digitalRead(stageDetectPin) == LOW) {
//    lcd.clear();  // clear the screen  
//    lcd.print("Chamber disconnected");
//
//    heatingAlgorithm.Reset();
//
//    analogWrite(heaterPin, 0);
//
//      Serial.print(elapsedSeconds, 1);
//    Serial.print("\t");
//    Serial.print("Chamber Disconnected");
//    Serial.print("\n"); 
// 
//    delay(1000);
//    return; // do not execute rest of loop
//  }

  sensor.measure(&temperature, &humidity, &dewpoint); // read the sensor. takes ~320ms measured using logic probe

  measuredTemp = temperature;  // change TYPE and store in PID variable

  if(abs(setpoint-measuredTemp) < 2 )  //check distance away from setpoint
  {  //we're close to setpoint, use conservative tuning parameters
    heatingAlgorithm.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     heatingAlgorithm.SetTunings(aggKp, aggKi, aggKd);
  }
  

  heatingAlgorithm.Compute();

  analogWrite(heaterPin, heaterPower); // Set the heater output to the calculated value

  elapsedSeconds = (millis() - startTime) / 1000.0;
  pwmPercent = 100.0 * (heaterPower / 255.0);

  // Display data on LCD
  lcd.clear();  // clear the screen  
  lcd.print(" Temp : ");
  lcd.print(temperature, 2);
  lcd.print(" C");

  lcd.setCursor(1,0); // move to second line, first position'
  lcd.print("Humid : ");
  lcd.print(humidity, 0);
  lcd.print("%");

  // Log total elapsed time, humidity, current temperature and heater power to serial port
  Serial.print(elapsedSeconds, 1);
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print(" %");
  Serial.print("\t");
  Serial.print(temperature, 2);
  Serial.print(" C");
  Serial.print("\t");
  Serial.print(pwmPercent , 2);
  Serial.print(" %");
  Serial.print("\t");
  Serial.print((heatingAlgorithm.ITerm/255) , 2);
  Serial.print("\n"); 

  delay(2900);  // should be at least 9x the length of time the sensor read time (i.e. keep sensor duty cycle < 10%)
}







