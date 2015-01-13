#include <Arduino.h>
#include <avr/io.h> 
#include <avr/interrupt.h> 

void setupPins()
{
  pinMode(statusA, INPUT);
  pinMode(statusB, INPUT);
  pinMode(statusI2C, INPUT);

  digitalWrite(statusA, LOW);
  digitalWrite(statusB, LOW);
  digitalWrite(statusI2C, LOW);

  digitalWrite(enableA, HIGH);
  digitalWrite(heaterA1, LOW);
  digitalWrite(heaterA2, LOW);
  pinMode(enableA, OUTPUT);   
  pinMode(heaterA1, OUTPUT);  
  pinMode(heaterA2, OUTPUT);

  digitalWrite(enableB, HIGH);
  digitalWrite(heaterB1, LOW);
  digitalWrite(heaterB2, LOW);
  pinMode(enableB, OUTPUT);   
  pinMode(heaterB1, OUTPUT);  
  pinMode(heaterB2, OUTPUT); 

  digitalWrite(valve1, HIGH);
  digitalWrite(valve2, LOW);  
  pinMode(valve1, OUTPUT);  
  pinMode(valve2, OUTPUT);
}

void splashScreen() {
  lcd.clear();
  lcd.print("-------------Aquinas");
  lcd.setCursor(1,0);
  lcd.print("kernel Controller v1");
  lcd.setCursor(2,5);
  lcd.print("DOI 1000.32/1000");
  lcd.setCursor(3,6);
  lcd.print("by CJQ at UCSD");
  delay(2000);
}

void drawScreen()
{
  lcd.clear();
  lcd.print("Air          Set");
  lcd.setCursor(1,0);
  lcd.print("Tub          Gas");
  lcd.setCursor(2,0);
  lcd.print("Hum          #A=");
  lcd.setCursor(3,0);
  lcd.print("Sub          #B=");
}

void drawValues()
{
  int pwmPercent;

  // Display the chamber air temp
  lcd.setCursor(0,4);
  lcd.print(tempAir,1);

  // Display the chamber temp and heater power
  lcd.setCursor(1,4);
  lcd.print(tempTopActual,1);
  lcd.print(" +");
  //  pwmPercent = heaterTop/2.55-1;
  pwmPercent = heaterTop+50;
  if (pwmPercent < 10) lcd.print("0");
  lcd.print(constrain(pwmPercent,0,99));

  // Display the chamber relative humidity and power
  lcd.setCursor(0,9);
  lcd.print(humidityActual,0);
  lcd.print("%");
  lcd.setCursor(2,4);
  lcd.print(tempWater,1);
  lcd.print(" +");
  //  pwmPercent = heaterWater/2.55-1;
  pwmPercent = heaterWater+50;
  if (pwmPercent < 10) lcd.print("0");
  lcd.print(constrain(pwmPercent,0,99));

  // Display the substage temp and power
  lcd.setCursor(3,4);
  lcd.print(tempSubActual,1);
  lcd.print(" +");
  //  pwmPercent = heaterSub/2.55;
  pwmPercent = heaterSub+50;
  if (pwmPercent < 10) lcd.print("0");
  lcd.print(constrain(pwmPercent,0,99));

  // port status globals are portA and portB
  // 0=ok, 1=not connected, 2=i2c error, 3=wrong device

  // display port A status
  lcd.setCursor(2,16);
  switch(portA){
  case 0: 
    lcd.print("Top");
    break;
  case 1:
    lcd.print("?Con");
    break;
  case 2:
    lcd.print("?I2C");
    break;
  case 4:
    lcd.print("!Sub");
  }

  lcd.setCursor(3,16);
  switch(portB){
  case 0: 
    lcd.print("Sub");
    break;
  case 1:
    lcd.print("?Con");
    break;
  case 2:
    lcd.print("?I2C");
    break;
  case 4:
    lcd.print("!Top");
  }

}

void checkPorts()
{
  int error;
  // port status globals are portA and portB
  // 0=ok, 1=not connected, 2=i2c error, 4=wrong device

  // Check Port A
  if ( digitalRead(statusA) == HIGH) portA=0;  // STATUS line pulled high by peripheral
  else {
    portA = 1; // 2-FAIL peripheral detected by i2c bus not ready
    //    recheckA=true;
  }

  // Check Port B
  // substage board doesn't pullup the status line!
  if (digitalRead(statusB) == HIGH) portB=0;   // STATUS line pulled high by peripheral
  else {
    portB = 1;
    //    recheckB=true;
  }

  // check if right device!
  if (portA==0) {
    //    recheckA=false; // dont recheck again until port is reset
    i2cBus.enableChannel1();
    Wire.beginTransmission(0x50);
    error = Wire.endTransmission();
    if (error == 0 ) portA=0; // found the DS28CM00 silicon serial number on the Stagetop incubator
  }

  if (portB==0) {
    //    recheckB=false; // dont recheck again until port is reset
    i2cBus.enableChannel0();
    Wire.beginTransmission(0x50);
    error = Wire.endTransmission();
    if (error == 0) portB=4; // found the DS28CM00 silicon serial number on the Stagetop incubator
  } 
}


// Turn on USART: setup the baud rate and serial interrupt
void setupSerial()
{
#define BAUD 9600
#include <util/setbaud.h>

  bitSet(UCSR0B, RXEN0);  // Enable reception
  bitSet(UCSR0B, TXEN0);  // Enable transmission
  bitSet(UCSR0B, RXCIE0); // Enable interrupt on reception

  // Set to character size to 8 bits 011 
  bitSet(UCSR0C, UCSZ00);
  bitSet(UCSR0C, UCSZ01);
  bitClear(UCSR0B, UCSZ02); 

  UBRR0L = UBRRL_VALUE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register 
  UBRR0H = UBRRH_VALUE; // Load upper 8-bits of the baud rate value into the high byte of the UBRR register 

#if USE_2X
  bitSet(UCSR0A, U2X0);
#else
  bitClear(UCSR0A, U2X0);
#endif

  interrupts(); // enable interrupts 
}

//ISR(USART_RX_vect)
//{ 
//  char ReceivedByte; 
//  ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived" 
//  UDR0 = ReceivedByte; // Echo back the received byte to the computer 
//}

int freeRam ()
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void latchFault() 
{
  analogWrite(heaterB1, 0);
  analogWrite(heaterB2, 0);
  analogWrite(heaterA1, 0);
  analogWrite(heaterA2, 0);

  while(true){
  };
}


