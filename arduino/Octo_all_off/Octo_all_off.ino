#include <Wire.h>
#include <LCDi2cNHD.h>    

LCDi2cNHD lcd = LCDi2cNHD(2,16,0x50>>1,0);
const int octoVPins[10] =  {11, 10, 2, 4, 6, 8, 3, 5, 7, 9};


void setup() {
    lcd.init();                          // Init the display, clears the display
  TWBR=152; /* == 50kHz SCL frequency */
   
  lcd.print("All valves ON");       // Classic Hello World!
   
  pinMode(2, OUTPUT); 
  pinMode(3, OUTPUT); 
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT); 
  pinMode(6, OUTPUT); 
  pinMode(7, OUTPUT); 
  pinMode(8, OUTPUT); 
  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT); 
  pinMode(11, OUTPUT); 
  
  digitalWrite(2, HIGH); 
  digitalWrite(3, HIGH); 
  digitalWrite(4, HIGH); 
  digitalWrite(5, HIGH); 
  digitalWrite(6, HIGH); 
  digitalWrite(7, HIGH); 
  digitalWrite(8, HIGH); 
  digitalWrite(9, HIGH); 
  digitalWrite(10, HIGH); 
  digitalWrite(11, HIGH); 
}
  
void loop() {
  }
  
