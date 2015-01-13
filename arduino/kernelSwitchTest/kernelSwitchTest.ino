#include <Wire.h>
#include <inttypes.h>
#include <LCDi2cNHD.h>

#include "kernelController.h"

LCDi2cNHD lcd = LCDi2cNHD(4,20,0x50>>1,0);  // 2 lines x 16 characters

void setup() {
   lcd.init();  
  TWBR=152; // Set I2C clock to 50kHz  
  lcd.setDelay(5,0);

  lcd.print("Aquinas Kernel");    
  lcd.setCursor(1,0);
  lcd.print("Power switch tester"); 
  lcd.setCursor(2,0);  
  lcd.print("July 09 2012 // CJQ");    

  delay(5000);
  
  /*
  pinMode(statusA, INPUT);
  pinMode(statusB, INPUT);
  pinMode(statusI2C, INPUT);
*/
  // pinMode(enableA, OUTPUT);   
  pinMode(heaterA1, OUTPUT);  
  pinMode(heaterA2, OUTPUT);
  // pinMode(enableB, OUTPUT);   
  pinMode(heaterB1, OUTPUT);  
  pinMode(heaterB2, OUTPUT); 
 /* 
  digitalWrite(enableB, LOW);
  digitalWrite(heaterB1, LOW);
  digitalWrite(heaterB2, LOW);

  pinMode(valve1, OUTPUT);  
  pinMode(valve2, OUTPUT);
  digitalWrite(valve1, LOW);
  digitalWrite(valve2, LOW);  
  
  delay(2000);
  */
}

void loop() { 
  lcd.clear();
  lcd.print("Heater A1&A2: ON");
  digitalWrite(heaterA1, HIGH);
  digitalWrite(heaterA2, HIGH);
  delay(4000);
 
  digitalWrite(heaterA1, LOW);  
  digitalWrite(heaterA2, LOW);

  lcd.clear();

  lcd.print("Heater B1&N2 : ON");
  digitalWrite(heaterB1, HIGH);
  digitalWrite(heaterB2, HIGH);
  delay(4000);
  digitalWrite(heaterB1, LOW);
  digitalWrite(heaterB2, LOW);  

  /*
  lcd.print("Valve 1 : ON");
  digitalWrite(valve1, HIGH);
  delay(2000);
  digitalWrite(valve1, LOW);  
  
      lcd.clear();
  lcd.print("Valve 2 : ON");
  digitalWrite(valve2, HIGH);
  delay(2000);
  digitalWrite(valve2, LOW);  
  */
}
