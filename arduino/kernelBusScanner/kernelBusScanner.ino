// --------------------------------------
// i2c_scanner
//
// This program (or code that looks like it)
// can be found in many places.
// For example on the Arduino.cc forum.
// The original author is not know.
//
// This sketch tests the standard 7-bit addresses
// from 0 to 127. Devices with higher bit address
// might not be seen properly.
//
// Adapted to be as simple as possible by Arduino.cc user Krodal
//
// June 2012
// Using Arduino 1.0.1
//

#include <PCA9540.h>
#include <LCDi2cNHD.h> 
#include <Wire.h>

const uint8_t statusI2C =  3;

const uint8_t statusA =  12;
const uint8_t statusB =  13;

const uint8_t enableA =  7;
const uint8_t enableB =  8;

const uint8_t busReadyA = A6; 
const uint8_t busReadyB = A7;

LCDi2cNHD lcd = LCDi2cNHD(4,20,0x50>>1,0);
PCA9540 i2cBus = PCA9540();

void setup()
{
  lcd.init();  
  TWBR=152; // Set I2C clock to 50kHz  
  lcd.setDelay(5,0);

  lcd.print("Aquinas Kernel");    
  lcd.setCursor(1,0);
  lcd.print("I2C bus scanner"); 
  lcd.setCursor(2,0);  
  lcd.print("July 09 2012 // CJQ");    

  Serial.begin(9600);
  Serial.println("\nKernel I2C bus scanner");

  delay(6000);

  //  KERNEL CONTROLLER - enable both ports
  pinMode(enableA, OUTPUT); // enableA
  pinMode(enableB, OUTPUT);  // enableB

  digitalWrite(enableA, HIGH);  //enableA
  digitalWrite(enableB, HIGH);  //enableB
  
  digitalWrite(statusA, LOW);  //enableB
  digitalWrite(statusA, LOW);  //enableB

  pinMode(statusA, INPUT);
  pinMode(statusB, INPUT);
  pinMode(statusI2C, INPUT );

//  digitalWrite(statusA, LOW);  //enableB
//  digitalWrite(statusA, LOW);  //enableB
//  digitalWrite(statusI2C, HIGH);  //enableA
}


void loop()
{
  
  bool portReady, portStatus, i2cStatus;
  Serial.println("Scanning Port A...");

  portReady = (analogRead(busReadyA) > 500);
  portStatus = (digitalRead(statusA)==HIGH);
  i2cStatus = (digitalRead(statusI2C)==HIGH);

  lcd.clear();
  //  lcd.print("Scanning Port A...");
  lcd.print("A:STAT  RDY    I2C: "); 
  lcd.setCursor(0,6);
  lcd.print(portStatus?"+":"0");
  lcd.setCursor(0,11);
  lcd.print(portReady?"+":"0");
  lcd.setCursor(0,19);
  lcd.print(i2cStatus?"+":"0");

  lcd.setCursor(1,0);

  i2cBus.enableChannel1();

  if (i2cScanner() == 0) {
    Serial.println("No I2C devices found on Port A\n");
    lcd.setCursor(3,0);
    lcd.print("No I2C devices");
  }
  else {
    Serial.println("Done with port A\n");
    lcd.setCursor(3,16);
    lcd.print("Done");
  }

  delay(3000);
  // 
  //  portReady = ( digitalRead(busReadyA)==HIGH ? true:false);
  //  portStatus = ( digitalRead(statusA)==HIGH ? true:false);
  //  i2cStatus = ( digitalRead(statusI2C)==HIGH ? true:false);
  //  

  portReady = (analogRead(busReadyB) > 500);
  portStatus = (digitalRead(statusB)==HIGH);
  i2cStatus = (digitalRead(statusI2C)==HIGH);


  lcd.clear();
  //          01234567890123456789
  lcd.print("B:STAT  RDY    I2C: ");    

  lcd.setCursor(0,6);
  lcd.print(portStatus?"+":"0");
  lcd.setCursor(0,11);
  lcd.print(portReady?"+":"0");
  lcd.setCursor(0,19);
  lcd.print(i2cStatus?"+":"0");

  //  lcd.setCursor(0,6);
  //  lcd.print(portReady);
  //  lcd.setCursor(0,11);
  //  lcd.print(portStatus);
  //  lcd.setCursor(0,19);
  //  lcd.print(i2cStatus);

  //lcd.print("Scanning Port B...");
  Serial.println("Scanning Port B...");
  lcd.setCursor(1,0);



  i2cBus.enableChannel0();

  if (i2cScanner() == 0) {
    Serial.println("No I2C devices found on Port B\n");
    lcd.setCursor(3,0);
    lcd.print("No I2C devices");
  }
  else {
    Serial.println("Done with port B\n");
    lcd.setCursor(3,16);
    lcd.print("Done");
  }

  delay(3000);           // wait 4 seconds for next scan
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
    if (address == 0x0C) address++; // This is the SMBus Alert Address

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (nDevices == 4) lcd.setCursor(2,0);
      if (nDevices == 8) lcd.setCursor(3,0);
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


