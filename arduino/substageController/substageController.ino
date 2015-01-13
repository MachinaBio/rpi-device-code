/*
substageController
 
 Biological Incubation // Controller
 Maintains heated environmental chamber to 37 C (fixed)
 
 Releases
 v1 - April 10 2011
 v2 - March 2012 - Tuned PID loops
 v3 - April 2012 - Added substage control
 
 Authors
 Carlo Quinonez
 Sidra Haiy
 
 This software is Copyright (c) 2011 The Regents of the University of California. All Rights
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
 
 // CJQ 2012-06-07: CHANGED MINIMUM POWER FOR STAGETOP ELEMENT TO 30/255

///////////////////////////////////////////////////
// VARIABLE DECLARATIONS are made prior to loading
// any external libraries, thus no external dependencies
///////////////////////////////////////////////////

// Variables for Sensirion SH75 temperature and humidity sensor. Type is set by the Sensirion library.
float topTemp, topHumid, topDew, botTemp, botHumid, botDew;

// Variables for PID control algorithm. Type is set by PID library.
double chamberTemp;	 // in degress Celcius
double chamberHumidity;	 // in percent RH
double chamberPower;  // from 0-355.
double substageTemp;  // in degress Celcius
double substagePower;  // from 0-355.

double setpoint = 37;  // in degrees Celcius

// Misc globals
unsigned long startTime;

///////////////////////////////////////////////////
// Externals libraries and pin assignments
//	are loaded after creating local dependancies (ie. pointers).
///////////////////////////////////////////////////
#include <Wire.h>
#include <inttypes.h>
#include <LCDi2cNHD.h>
#include <Sensirion.h>
#include <PID.h>

#include "substageController.h"

///////////////////////////////////////////////////
// Create control objects for hardware.
///////////////////////////////////////////////////
LCDi2cNHD lcd = LCDi2cNHD(2,16,0x50>>1,0);	// 2 lines x 16 characters
Sensirion chamberSensor = Sensirion(chamberDataPin, chamberClockPin); 
Sensirion substageSensor = Sensirion(substageDataPin, substageClockPin); 
PID chamberAlgorithm = PID(&chamberTemp, &chamberPower, &setpoint, aggKp, aggKi, aggKd, DIRECT); 
PID substageAlgorithm = PID(&substageTemp, &substagePower, &setpoint, ssKp, ssKi, ssKd, DIRECT); 

///////////////////////////////////////////////////
// One time setup
///////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);

  chamberSensor.writeSR(LOW_RES);

  chamberPower	= 0;
  substagePower = 0;

  pinMode(chamberHeaterPin, OUTPUT);  
  pinMode(substageHeaterPin, OUTPUT);  
  pinMode(chamberDetectPin, INPUT);
  pinMode(substageDetectPin, INPUT);

  // turn off internal pull-ups on DetectPins
  digitalWrite(chamberDetectPin, LOW);
  digitalWrite(substageDetectPin, LOW);

  lcd.init();
  lcd.setCursor(1,0);  // hide cursor
  lcd.blink_off(); 

  startTime = millis();	 // store start time in milliseconds

 // CJQ 2012-06-07: CHANGED MINIMUM POWER FOR STAGETOP ELEMENT TO 30/255
 // Trying to compensate for cold tub by manually boosting lower limit. I believe convection from the
 // substage element is raising the air temp in the chamber, independent of the tub temperature. However
 // the sample temperature is modulate by BOTH tub and air temp... I think!
 
    chamberAlgorithm.SetOutputLimits(20, 255); 
  chamberAlgorithm.SetSampleTime(3000);
  chamberAlgorithm.SetMode(AUTOMATIC);

  substageAlgorithm.SetOutputLimits(0, 99); 
  substageAlgorithm.SetSampleTime(3000);
  substageAlgorithm.SetMode(AUTOMATIC);

}

///////////////////////////////////////////////////
// Processed continuously.
///////////////////////////////////////////////////
void loop() {
  double elapsedSeconds;
  float pwmPercent;
  uint8_t stageStatus;


  chamberSensor.measure(&topTemp, &topHumid, &topDew); // read the sensor. takes ~320ms measured using logic probe 
  substageSensor.measure(&botTemp, &botHumid, &botDew); // read the sensor. takes ~320ms measured using logic probe
  
  chamberSensor.readSR(&stageStatus);

  // transfer measured values to PID
  chamberTemp = topTemp;
  chamberHumidity = topHumid;
  substageTemp = botTemp; 

  if(abs(setpoint-chamberTemp) < 2 )  //check distance away from setpoint
  {	 //we're close to setpoint, use conservative tuning parameters
    chamberAlgorithm.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    chamberAlgorithm.SetTunings(aggKp, aggKi, aggKd);
  }


  chamberAlgorithm.Compute();
  substageAlgorithm.Compute();

  analogWrite(chamberHeaterPin, chamberPower); // Set the heater output to the calculated value

  elapsedSeconds = (millis() - startTime) / 1000.0;
  pwmPercent = 100.0 * (chamberPower / 255.0);

  ////////////////////////////
  // Display data on LCD
  lcd.clear();	// clear the screen	 
  if ( digitalRead(chamberDetectPin) == LOW) {
    lcd.print("NoChamber");
  }
  else {
    lcd.print("Top ");
    lcd.print(chamberTemp,1);
    lcd.print("C ");
    lcd.print(chamberHumidity, 0);
    lcd.print("% ");
  }	 

  lcd.setCursor(1,0); // move to second line, first position

  if ( digitalRead(substageDetectPin) == LOW) {
    lcd.print("NoStage");
  }
  else {
    lcd.print("Bot ");
    lcd.print(substageTemp,1);
    lcd.print("C ");
  }

  lcd.setCursor(1,10);
  lcd.print(pwmPercent,0);
  lcd.print(" ");
  lcd.print(substagePower,0);

  ////////////////////////////
  // Log debug data to serial port
  Serial.print("Time: ");
  Serial.print(elapsedSeconds, 1);
  Serial.print("s \tChamber: \t");
  if ( digitalRead(chamberDetectPin) == LOW) {
    Serial.print("NC\t\t\t");
  }
  else {
    Serial.print(chamberTemp, 2);
    Serial.print(" C \t");
    Serial.print(chamberHumidity, 1);
    Serial.print(" % RH \t");
    Serial.print(pwmPercent, 1);
    Serial.print(" % pow \t");
    Serial.print(stageStatus, BIN);
    Serial.print(" Status Register \t");
  }	 

  Serial.print("Substage: \t");
  if ( digitalRead(substageDetectPin) == LOW) {
    Serial.print("NC\t\t");
  }
  else {
    Serial.print(substageTemp, 2);
    Serial.print(" C \t");
    Serial.print(substagePower, 0);
    Serial.print(" % pow");
  }	 
  Serial.print("\n"); 
  substageHeating();  
}

void substageHeating() {
  // check if heater connected

  if ( digitalRead(substageDetectPin) == LOW) {
    substageAlgorithm.SetMode(MANUAL);
    digitalWrite(substageHeaterPin, LOW);
    delay(2900);
  }
  else {
    substageAlgorithm.SetMode(AUTOMATIC);
    int onTime = substagePower * 29;
    int offTime = 2900 - onTime; // total time (i.e. 2900 ms) should be longer than 10x sensor read time to prevent self-heating artifacts

    digitalWrite(substageHeaterPin, HIGH);
    delay(onTime);
    digitalWrite(substageHeaterPin, LOW);
    delay(offTime);
  }
}







