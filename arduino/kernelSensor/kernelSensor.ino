#include <Wire.h>
#include <inttypes.h>

#include <PCA9540.h>
#include <LCDi2cNHD.h> 
#include <SE95.h>
#include <SensirionI2C.h>
#include "kernelController.h"

LCDi2cNHD lcd = LCDi2cNHD(4,20,0x50>>1,0);
PCA9540 i2cBus = PCA9540();
SE95 sensor0x48 = SE95(0x48);
SE95 sensor0x49 = SE95(0x49);
SensirionI2C sht75 = SensirionI2C(dataPin, clockPin); 

void setup() {
  lcd.init();  
  TWBR=152; // Set I2C clock to 50kHz  
  lcd.setDelay(5,0);

  lcd.print("Aquinas Kernel");    
  lcd.setCursor(1,0);
  lcd.print("I2C sensor test"); 
  lcd.setCursor(2,0);  
  lcd.print("July 11 2012 // CJQ");    

  delay(2000);
  
  pinMode(statusA, INPUT);
  pinMode(statusB, INPUT);
  pinMode(statusI2C, INPUT);

  pinMode(enableA, OUTPUT);   
  pinMode(enableB, OUTPUT);   
  pinMode(heaterA1, OUTPUT);  
  pinMode(heaterA2, OUTPUT);
  pinMode(heaterB1, OUTPUT);  
  pinMode(heaterB2, OUTPUT);
  
  pinMode(valve1, OUTPUT);  
  pinMode(valve2, OUTPUT);

  digitalWrite(heaterA1, LOW);
  digitalWrite(heaterA2, LOW);
  digitalWrite(heaterB1, LOW);
  digitalWrite(heaterB2, LOW);
  digitalWrite(valve1, LOW);
  digitalWrite(valve2, LOW);  
  
  digitalWrite(enableA, HIGH);
  digitalWrite(enableB, HIGH);
  
 i2cBus.enableChannel1();
}

void loop() {
  float temp0x48, temp0x49, tempSHT75, rhSHT75, dewSHT75;
/*  
  lcd.clear();
  lcd.print("Reading Port A...");
  
  delay(300);
*/  
  lcd.clear();
  lcd.print("Port A PCB=>top/sub");
  lcd.setCursor(1,7); // leave room for "12.34C "
  lcd.print("0x48 hum/ssg");
  lcd.setCursor(2,7); // leave room for "12.34C "
  lcd.print("0x49 cul/obj");
  lcd.setCursor(3,15);
  lcd.print("SH75"); // leave room for  "12.34C-12%-12C"
  
  
  temp0x48 = sensor0x48.temp();
  temp0x49 = sensor0x49.temp();
  
  tempSHT75 = 0;
  rhSHT75 = 0;
  dewSHT75 = 0;
  
  sht75.measure(&tempSHT75, &rhSHT75, &dewSHT75);
  
  lcd.setCursor(1,0);
  lcd.print(temp0x48, 2);

  lcd.setCursor(2,0);
  lcd.print(temp0x49, 2);

  lcd.setCursor(3,0);
  lcd.print(tempSHT75, 2);
  lcd.print("C-");
  lcd.print(rhSHT75, 0);
  lcd.print("%-");
  lcd.print(dewSHT75, 0);
  
  delay(100);
}
