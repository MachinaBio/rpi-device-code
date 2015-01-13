#include <Wire.h>
#include <inttypes.h>
#include <LCDi2cNHD.h>
#include <PCA9540.h>
#include <LTC1695.h>

#include "kernelController.h"

LCDi2cNHD lcd = LCDi2cNHD(4,20,0x50>>1,0);  // 2 lines x 16 characters
LTC1695 fan = LTC1695();
PCA9540 i2cBus = PCA9540();

void setup() {
  lcd.init();  
  TWBR=152; // Set I2C clock to 50kHz  
  lcd.setDelay(5,0);

  lcd.print("Aquinas Kernel");    
  lcd.setCursor(1,0);
  lcd.print("Fan tester"); 
  lcd.setCursor(2,0);  
  lcd.print("July 11 2012 // CJQ");    

  delay(4000);
  
  pinMode(7, OUTPUT); // enableA
  pinMode(8, OUTPUT);  // enableB
      
  digitalWrite(7, HIGH);  //enableA
  digitalWrite(8, HIGH);  //enableB
}

void loop() { 
   byte error;
    
  lcd.clear();
  lcd.print("Scanning Port B...");
  lcd.setCursor(1,0);
 
 i2cBus.enableChannel0();
 
 if (i2cScanner() == 0) {
    lcd.setCursor(3,0);
    lcd.print("No I2C devices");
 }
  else {
    lcd.setCursor(3,0);
    lcd.print("Done with port B");
  }

  delay(3000);           // wait 4 seconds for next scan
  
/*  lcd.clear();
  lcd.print("Port A : BST->10%");
  i2cBus.enableChannel1();
  fan.adjustSpeed(7);
  delay(3000);
 
  lcd.clear();
  lcd.print("Port A : ->50%");
  fan.halfSpeed();
  delay(3000);
  
  lcd.clear();
  lcd.print("Port A : ->100%");
  fan.fullSpeed();  */
 
  lcd.clear();
  lcd.print("Port B : BST->10%");
  error = fan.adjustSpeed(7);
  lcd.setCursor(1,0);
  lcd.print("Return value : ");
  lcd.print(error);
            lcd.setCursor(2,0);
  lcd.print("Current speed : ");
  lcd.print(fan.getSpeed());
    delay(3000);
  
  lcd.clear();
  lcd.print("Port B : ->50%");
  //fan.halfSpeed();
  error = fan.adjustSpeed(60);
    lcd.setCursor(1,0);
  lcd.print("Return value : ");
  lcd.print(error);
            lcd.setCursor(2,0);
    lcd.print("Current speed : ");
  lcd.print(fan.getSpeed());
  delay(3000);
  
  lcd.clear();
  lcd.print("Port B : ->100%");
  error = fan.fullSpeed();
      lcd.setCursor(1,0);
  lcd.print("Return value : ");
  lcd.print(error);
          lcd.setCursor(2,0);
    lcd.print("Current speed : ");
  lcd.print(fan.getSpeed());
  delay(3000);

  lcd.clear();
  error = lcd.print("Port B : STOP");
  fan.fullSpeed();
      lcd.setCursor(1,0);
  lcd.print("Return value : ");
  lcd.print(error);
        lcd.setCursor(2,0);
  lcd.print("Current speed : ");
  lcd.print(fan.getSpeed());
  delay(3000);
/*
  i2cBus.enableChannel1();
  lcd.clear();
  lcd.print("Port A : STOP");
  fan.fullSpeed();
  delay(3000);  
  */
}



int i2cScanner () {
  byte error, address;
  int nDevices;

  nDevices = 0;
  for(address = 0; address <= 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    if (address == 0x0C) continue; // This is the SMBus Alert Address
    
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (nDevices == 4) lcd.setCursor(2,0);
      lcd.print("0x");
      if (address<16) {
        Serial.print("0");
        lcd.print("0");
      }
      Serial.print(address,HEX);
      Serial.println(" !");

      lcd.print(address, HEX);
      lcd.print(",");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (nDevices == 4) lcd.setCursor(2,0);
      lcd.print("?x");
      if (address<16) {
        Serial.print("0");
        lcd.print("0");
      }
      Serial.println(address,HEX);
      Serial.println(" !");
      
      lcd.print(address, HEX);
      lcd.print(",");

      nDevices++;
    }    
  }
  return nDevices;
}
