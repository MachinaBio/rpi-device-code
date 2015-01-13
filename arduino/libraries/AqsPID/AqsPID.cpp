/**********************************************************************************************
 * Aquinas PID Library - Version 1
 * by Carlo Quinonez, July 11 2012
 * adapted from Arduino PID by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Code is licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.
 **********************************************************************************************/

#include <Arduino.h>
#include <AqsPID.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
AqsPID::AqsPID()
{
	AqsPID::setLimits(0.0, 255.0, 50.0);		//default output limit corresponds to 
												//the arduino pwm limits

    sampleTime = 1000;							//default Controller Sample Time is 1 seconds

    AqsPID::setDirection(DIRECT);
    AqsPID::setTunings(0.0, 0.0, 0.0);

    lastTime = millis();				
    output = 0.0;
    setpoint = 0.0;
    Iterm = 0.0;
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 **********************************************************************************/ 
int AqsPID::updateNow() {
   long elapsedTime; 
   elapsedTime = (millis() - lastTime);
   if ( elapsedTime>=sampleTime ) return elapsedTime;
   return 0;
}

void AqsPID::calculate(long elapsedTime) {
    float correctionFactor, error, Dterm, Pterm;
    
    correctionFactor = elapsedTime / sampleTime;
    error  = setpoint - input;

    Pterm  = kp * error; 
    Iterm += ki * error * correctionFactor;   
    Dterm  = kd * (lastInput - input) / correctionFactor;

    checkIterm();
    
    output = Pterm + Iterm - Dterm;
    
    checkOutput();
    
    lastInput = input;
    lastTime = millis();
    }   

float AqsPID::getOutput() {
    long elapsedTime;
    
    elapsedTime = updateNow();
    if (elapsedTime) return output;
    
    calculate(elapsedTime);
    
    return output;
    }

void AqsPID::checkIterm(){
    if (Iterm >	Imax) Iterm = Imax;
      else if (Iterm < -Imax) Iterm = -Imax;
    }

void AqsPID::checkOutput(){
    if (output > outMax) output = outMax;
      else if (output < outMin) output = outMin;
   }


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void AqsPID::setTunings(float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp;
   dispKi = Ki;
   dispKd = Kd;
   
   float sampleTimeInSec = ((float)sampleTime)/1000;  
   kp = Kp;
   ki = Ki * sampleTimeInSec;
   kd = Kd / sampleTimeInSec;
 
  if(direction == REVERSE) {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
      }
   }
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void AqsPID::setSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)sampleTime;
      ki *= ratio;
      kd /= ratio;
      sampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void AqsPID::setLimits(float Min, float Max, float iLimit)
{
   if(Min >= Max) return;
   outMin = Min; 
   outMax = Max;
   Imax = iLimit;
 
   if(output > outMax) output = outMax;
     else if(output < outMin) output = outMin;

   if(Iterm > outMax) Iterm= outMax;
     else if(Iterm < outMin) Iterm= outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void AqsPID::setDirection(int newDirection)
{
   if( newDirection != direction)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   direction = newDirection;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float AqsPID::getKp(){ return  dispKp; }
float AqsPID::getKi(){ return  dispKi;}
float AqsPID::getKd(){ return  dispKd;}
int AqsPID::getDirection(){ return direction;}

