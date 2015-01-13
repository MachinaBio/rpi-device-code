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

  digitalWrite(valve1, LOW);
  digitalWrite(valve2, LOW);  
  pinMode(valve1, OUTPUT);  
  pinMode(valve2, OUTPUT);
}

float readCurrentCounts(int analogPin)
{
  // we have to use the 1.1V reference - make sure it's set before this routine is called!
  long int accumulator;
  float returnVal;
 
//  const int samples = 1000; // max sample rate is ~8.9kSamples/sec. So take 1000 samples to average out AC ripple

  analogRead(analogPin); // discard result. Aref is only changed during analogRead, so make sure it is set now.
  delay(5); // let Aref settle for 5 ms. The external 0.1uf cap takes that long to discharge from 5V->1.1V

  accumulator=0; 
  for (int i=0; i< 1000; i++)
  {
    accumulator += analogRead(analogPin);
  }
  
  returnVal = (float) accumulator / 1000.0; // divide by samples, rounding down
  return returnVal;
}

void splashScreen() {
 #if LOGDATA 
  lcd.clear();
//           12345678901234567890
  lcd.print("-----------Incubator");
  lcd.setCursor(1,0);
//           12345678901234567890  
  lcd.print("       BT Datalogger");
  lcd.setCursor(2,0);
//           12345678901234567890
  lcd.print("       DOI xx.yy/nnn");
  lcd.setCursor(3,0);
//           12345678901234567890
  lcd.print("      by CJQ at UCSD");
 #else 
  lcd.clear();
//           12345678901234567890
  lcd.print("-----------Incubator");
  lcd.setCursor(1,0);
//           12345678901234567890  
  lcd.print(" BT Stackcontroller");
  lcd.setCursor(2,0);
//           12345678901234567890
  lcd.print("       DOI xx.yy/nnn");
  lcd.setCursor(3,0);
//           12345678901234567890
  lcd.print("      by CJQ at UCSD");
 #endif
 
  delay(2000);
}

void drawScreen()
{
  lcd.clear();
  lcd.print("Air=     PWM Amp Gas");
  lcd.setCursor(1,0);
  lcd.print("Tub=");
  lcd.setCursor(2,0);
  lcd.print("Hum=");
  lcd.setCursor(3,0);
  lcd.print("Sub=         Vcc=");
}

void drawValues()
{
  int pwmPercent;

  ////////////////////////////////////////////////////////////////////////////
  // Display the chamber air temp
  lcd.setCursor(0,4);
  lcd.print(tempAir,1);

  ////////////////////////////////////////////////////////////////////////////  
  // Display the CHAMBER temp and heater power and current
  lcd.setCursor(1,4);
  lcd.print(tempTopActual,1);
  lcd.print("-");

  pwmPercent = heaterTop+500.;  // rescale to print 3 sig figures  
  if (pwmPercent < 10) lcd.print("0");
  if (pwmPercent < 100) lcd.print("0");
  lcd.print(constrain(pwmPercent,0,999));
  lcd.print("-");

  if (chamberHtrAmps < 10) lcd.print("0");
  if (chamberHtrAmps < 100) lcd.print("0");
  lcd.print(constrain(chamberHtrAmps,0.0,999.0), 0);

  ////////////////////////////////////////////////////////////////////////////
  // Display the CHAMBER relative humidity
  lcd.setCursor(2,17);
  lcd.print(humidityActual,0);
  lcd.print("%");

  ////////////////////////////////////////////////////////////////////////////
  // Display the HUMIDIFICATION heater power and amps
  lcd.setCursor(2,4);
  lcd.print(tempWater,1);
  lcd.print("-");

  pwmPercent = heaterWater+500;
  if (pwmPercent < 10) lcd.print("0");
  if (pwmPercent < 100) lcd.print("0");
  lcd.print(constrain(pwmPercent,0,999));
  lcd.print("-");

  if (waterHtrAmps < 10) lcd.print("0");
  if (waterHtrAmps < 100) lcd.print("0");
  lcd.print(constrain(waterHtrAmps,0.0,999.0), 0 );

  ////////////////////////////////////////////////////////////////////////////
  // Display the SUBSTAGE temp and power
  lcd.setCursor(3,4);
  lcd.print(tempSubActual,1);
  lcd.print("-");

  pwmPercent = heaterSub+500;
  if (pwmPercent < 10) lcd.print("0");
  if (pwmPercent < 100) lcd.print("0");
  lcd.print(constrain(pwmPercent,0,999));

  ////////////////////////////////////////////////////////////////////////////
  // Display VCC reading
  lcd.setCursor(3,17);
  if (htrSupplyVolts < 10) lcd.print("0");
  if (htrSupplyVolts < 100) lcd.print("0");
  lcd.print(constrain(htrSupplyVolts,0.0,999.0), 0);
}

void checkPorts()
{
  int error;
  // port status globals are portA and portB
  // 0= ALL OK
  // 1= NO CONNECTION
  // 2= BUS EEROR
  // 4= WRONG DEVICE

  // Has Port A status line has been pulled him by a peripheral?
  if ( digitalRead(statusA) == HIGH) { // Connected!
    portA=4;  // set WRONG DEVICE by default
    i2cBus.enableChannel1(); // connect to portA i2c bus
    Wire.beginTransmission(0x50); // ping for the DS28CM00 silicon serial number 
    error = Wire.endTransmission();
    if (error == 0 ) portA=0; // Found DS28CM00 => Stagetop. Set AOK flag
  }
  else {
    portA = 1; // set NO CONNECTION flag
  }

  // Has Port B status line has been pulled him by a peripheral?
  if (digitalRead(statusB) == HIGH) { // Connected!
    portB=4;   // set WRONG DEVICE by default 
    i2cBus.enableChannel0(); // connect to portB i2c bus
    Wire.beginTransmission(0x50); // ping for the DS28CM00 silicon serial number
    error = Wire.endTransmission();
    if (error != 0) portB=0; // No DS28CM00 => Substage. Set AOK flag
  }
  else {
    portB = 1; // set  NO CONNECTION flag
  }
}

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

  while(true){};
}


#if LOGDATA //  throws error because this collides with the Arduino Serial interupt handler
#else
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

ISR(USART_RX_vect)
{ 
  char ReceivedByte; 
  ReceivedByte = UDR0; // Fetch the recieved byte value into the variable "ByteReceived" 
  UDR0 = ReceivedByte; // Echo back the received byte to the computer 
}
#endif


float readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  
  long int accumulator;
  float returnVal;
  
  accumulator=0; 
  for (int i=0; i< 1000; i++)
  {
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring
     
    accumulator += ADC;
  }
  
  returnVal = (float) accumulator / 1000.0; // divide by samples, rounding down
  return returnVal;
}

