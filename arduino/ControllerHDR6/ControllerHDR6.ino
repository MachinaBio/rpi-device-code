/*
conOcto
 
 Control Stack // Octo valve manifold.
 controls pressure reservoir and eight solenoids 
 
 See http://www.arduino.cc/en/Tutorial/SerialCallResponseASCII
 
 Releases
 v1 - April 5 2011
 v2 - July 9, 2011 : Changed valve numbering
 v3 - September 11, 2011 : Changed PID tuning
 
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
// Externals libraries and pin assignments
//  are loaded after creating local dependancies (ie. pointers).
///////////////////////////////////////////////////
#include <Wire.h>
#include <inttypes.h>
#include <math.h>
/* Ideally the LCDi2cNHD and PID libraries would be loaded from the classes
 that use require them, but the current arduino IDE (v22) doesn't allow libaries to include
 other libraries (i.e. nested libraries) */
#include <LCDi2cNHD.h>
#include <PID.h>
#include <Aquinas.h>
#include <CommandBus.h>
#include <PressureReservoirHDR.h>
#include <Display.h>

#include "conOcto.h"

///////////////////////////////////////////////////
// Create control objects for hardware.
///////////////////////////////////////////////////

PressureReservoir octoReservoir = PressureReservoir(octoPresureSensor, octoReservoirConfig);
CommandBus vsNetwork = CommandBus(moduleAddress, octoNValves);
Display octoDisplay = Display(octoLCD, moduleAddress);  // 2 lines x 16 characters

///////////////////////////////////////////////////
// One time setup
///////////////////////////////////////////////////
void setup() {
  // Set all the valves to OFF before setting them as outputs
  for(int i=0; i++; i<octoNValves) { 
    digitalWrite(octoVPins[i], LOW); // Write the valve
    pinMode(octoVPins[i], OUTPUT); 
  } // Write the valve
  
  char addressList[17]="ABCDEFGHIJKLMNOP";
  int addressSwitches[4]={A1, A2, A3, A6};
  int addressIndex=0;
  
  for (int i=0; i<4; i++)
    if (analogRead(addressSwitches[i])>512)
      addressIndex += pow(2,i);
  
  vsNetwork.moduleAddress = addressList[addressIndex];
  octoDisplay.moduleAddress = vsNetwork.moduleAddress;
  
  octoDisplay.begin();
  vsNetwork.begin();
  
  // hack for valves 3+4, the other two capilary control valves
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
}

///////////////////////////////////////////////////
// Processed continuously.
///////////////////////////////////////////////////
void loop() {  
  boolean valveStates[octoNValves];

  /* Handle network communications. This also brings the valve register up to date. */
  vsNetwork.update();
  
  /* Set the valves according to the valve registers from network object */
  for(int i=0; i<octoNValves; i++) { 
    valveStates[i] = vsNetwork.valveRegister[i];
    if (valveStates[i]) { 
      digitalWrite(octoVPins[i], HIGH); 
    } 
    else { 
      digitalWrite(octoVPins[i], LOW); 
    }
  }

  /*  Ask the reservoir to update itself. The reservoir is smart enough
   to return immediately without updating if it's been less than 100msec
   since the last update */
  octoReservoir.setpoint = vsNetwork.setpointRegister;
  octoReservoir.update();

  /* Update the display */
  octoDisplay.setValves(valveStates);
  octoDisplay.currentPressure=octoReservoir.currentPressure;
  octoDisplay.setPressure=octoReservoir.setpoint;
  octoDisplay.update();

  /* flash the screen if the network has failed */
 // if (vsNetwork.failsafe() ) {
 //   octoDisplay.flash();
 // }

}



























