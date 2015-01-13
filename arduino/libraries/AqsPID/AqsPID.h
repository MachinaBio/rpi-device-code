#ifndef AqsPID_h
#define AqsPID_h

//Constants used in some of the functions below
const int DIRECT    = 0;
const int REVERSE   = 1;

class AqsPID
{


  public: 
  //commonly used functions **************************************************************************
    AqsPID();                             //   Constructor. Initial tuning parameters.
	
    void setInput(float);                 //   Inputs a new data points
    
    float getOutput(void);                //   Returns the error signal. If the calculation cycle time
                                          //   has elapsed, a PID calculation is performed to determine
                                          //   the error signal. Otherwise, this returns the old error
                                          //   signal. ON/OFF and calculation frequency can be set using
                                          //   setMode and setSampleTime respectively.

    void setLimits(float, float, float);  //   Clamps the output and I terms to a specific range.
                                          //   0-255  by default, but it's likely the user will want
                                          //   to change this depending on the application
                                          
    int updateNow(void);                  //   Checks if the PID sample time has elapsed and a new
                                          //   error signal is ready for retrieval. If the time has
                                          //   elapsed, then this returns the elapsed time in 
                                          //   milliseconds (any non-zero int is TRUE). If the time
                                          //   hasn't elapsed, then it returns 0 (0 is FALSE).
                                          
    void setSetpoint(float);              //   Changes the setpoint.
	
  //available but not commonly used functions ********************************************************
    void setTunings(float, float, float); // * While most users will set the tunings once in the 
                                    	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
                                          
	void setDirection(int);	  // * Sets the Direction, or "Action" of the controller. 
	                                      //   DIRECT means the output will increase when error
	                                      //   is positive.  REVERSE means the opposite.  it's very
	                                      //   unlikely that this will be needed once it is set in
	                                      //   the constructor.
										  
    void setSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	float getKp();						  // These functions query the pid for interal values.
	float getKi();						  //  they were created mainly for the pid front-end,
	float getKd();						  // where it's important to know what is actually 
	float getSetpoint();				  //  inside the PID.
	int getDirection();					  //

  private:
	
	float setpoint;
	float output;
	
	float dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	float dispKi;				//   format for display purposes
	float dispKd;				//
    
	float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter

	int direction;

	unsigned long lastTime;
    float Iterm;
	float input, lastInput;		

	int sampleTime;
	
	float outMin, outMax, Imax;
	
    void calculate(long);
    void checkIterm();
    void checkOutput();
};
#endif

