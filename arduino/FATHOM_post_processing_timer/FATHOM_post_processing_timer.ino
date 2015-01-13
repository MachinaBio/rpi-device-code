/*
 # Product: FATHON support removal timer
 # Description:     
 #    This code is cycles a 16A relay on for 5 min and off for 45min (1:9 duty cycle).
 #    It's intended to control excessive heating in the post processing support removal
 #    machine.
 #    
 #    Adapted from http://www.dfrobot.com/wiki/index.php/16A_Relay_Module(SKU:DFR0251)
 #
 # Link:
 #    D     --  2       // Signal
 #    GND   --  GND 
 #    VCC   --  VCC
 #    COM   --  AC      // INPUT 
 #    NO    --  AC      // OUTPUT
 #    LED - --  GND
*/
   
 
 
int Relay = 3;
long int OnTime = 60000; // 1 mins in milliseconds
long int OffTime = 540000; // 9 mins in milliseconds
  
void setup()
{
  pinMode(13, OUTPUT);         //Set Pin13 as output
  digitalWrite(13, HIGH);     //Set Pin13 High
  pinMode(Relay, OUTPUT);     //Set Pin3 as output
}
void loop()
{
          digitalWrite(Relay, HIGH);   //Turn off relay
          delay(OnTime);
          digitalWrite(Relay, LOW);    //Turn on relay
          delay(OffTime);
}
