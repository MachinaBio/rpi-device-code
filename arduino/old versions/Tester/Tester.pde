#include <Wire.h>
#include <inttypes.h>
#include <LCDi2cNHD.h> 
#include <Display.h>
#include <CommandBus.h>

const LCDconfig octoLCD = {
  2,   // number of rows
  16,   // number of columns
  0x50 };  // I2C address

char moduleAddress = 'B';

Display octoDisplay = Display(octoLCD, moduleAddress);  // 2 lines x 16 characters
CommandBus vsNetwork = CommandBus(moduleAddress, 8);

long int starttime= millis();
long int lastdebug = millis();

void setup() {
  octoDisplay.begin();
  vsNetwork.begin();
}

void loop() {
  //  Serial.println("Loop");

  boolean valves[8];
  octoDisplay.update();
  vsNetwork.update();
  
  for (int i=0; i<8; i++) valves[i]=vsNetwork.valveStates[i];
//  vsNetwork.getValves(valves);

  //  if (millis() > starttime + 4000) 
  // boolean valves[8]  = {
  //      true,false,true,true,true,true,true,true        };
  octoDisplay.setValves(valves);

  if(vsNetwork.valid) { 
      octoDisplay.currentPressure=10;
      octoDisplay.setPressure=9;}

  if (millis() > (lastdebug + 2000)){
    lastdebug = millis();
    Serial.println("Printing valve states");
    for (int i=0; i<8; i++) {
      if (valves[i]) Serial.print(i);
      else Serial.print('o');
    }
    Serial.print("\n");
  }
}




