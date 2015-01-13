/**
 * A hardware abstraction class for a pressure reservoir.
 *
 * This class implements a PID controller.
 *
 * @file Reservoir.H
 * @author Carlo Quinonez <cquinonez@ucsd.edu>
 * @version 1.0
 * @section LICENSE
 *This software is Copyright © 2011 The Regents of the University of California. All Rights
 *Reserved.
 *
 *Permission to use, copy, modify, and distribute these design files, software, firmware and
 *documentation for educational, research and non-profit purposes, without fee, and without
 *a written agreement is hereby granted, provided that the above copyright notice, this
 *paragraph and the following three paragraphs appear in all copies.
 *
 *Permission to make commercial use of this software may be obtained by contacting:
 *Technology Transfer Office
 *9500 Gilman Drive, Mail Code 0910
 *University of California
 *La Jolla, CA 92093-0910
 *(858) 534-5815
 *invent@ucsd.edu
 *
 *These design files, software, firmware and documentation are copyrighted by The Regents
 *of the University of California. The software program and documentation are supplied "as is",
 *without any accompanying services from The Regents. The Regents does not warrant that the
 *operation of the program will be uninterrupted or error-free. The end-user understands that
 *the program was developed for research purposes and is advised not to rely exclusively on
 *the program for any reason.
 
 *IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
 *SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
 *USE OF THESE DESIGN FILES, SOFTWARE, FIRMWARE AND DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 *CALIFORNIA HAS BEEN ADVISED OFTHE POSSIBILITY OF SUCH DAMAGE. THE UNIVERSITY OF CALIFORNIA
 *SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 *ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATIONS TO PROVIDE
 *MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.  
 */
#ifndef VSRESERVOIR
#define VSRESERVOIR

// **************************************************************************
// 8/5/2012, CJQ - Patched to compile with Arduino 1.0, 
//                            as well as older Arudino (e.g. 0023)
// **************************************************************************
#if (ARDUINO >= 100)
	#include <Arduino.h>
#else
	#include "WConstants.h"
#endif

#ifndef PID_v1_h
#error The PID library must be included from your sketch.
#endif

class PressureReservoir
{  
public:
	/**
	 * Setpoint in PSIG for the reservoir.
	 * 
	 */
	double setpoint;         // in PSI
	
	/**
	 * Current reservoir pressure in PSIG.
	 * 
	 */
	double real;  // in PSI
	
	PressureSensor sensor;
	double controlOutput;     // from -255 to 255
	long int nextUpdateTime;
	ReservoirConfig reservoir;
	PID algorithm;
	long  rawSensor;
	
	/**
	 * CONSTRUCTOR
	 *
	 * Constructor initializes the PID control algorithim and sets relevant pin identities.
	 * 
	 * @param setpoint The setpoint of the reservoir in PSI
	 * @param sensor Low-level parameters for pressure sensor
	 * @param tuning Low-level parameters for pressure controller
	 */
	PressureReservoir(ReservoirConfig newReservoir, PressureSensor newSensor, AquinasGlobal* address)
	:  algorithm(	&real,	 // call the constructor for algorithim object
				 &controlOutput,
				 &setpoint, 
				 0 ,							// default values only - actual values set in constructor
				 0, 							
				 0, 							
				 DIRECT),
    sensor(newSensor),    	  	// call the constructor for sensor object
    controlOutput(0), 				// set Error Output to 0 (no flow in or out
    real(0),			// clear the current pressure
    setpoint(0),							// clear the setpoiint
    reservoir(newReservoir)
	{
		global=address;
			// Set frequency for pwmValve to ~32Hz
		
			//		TCCR1B = TCCR1B & 0b11111000 | 0x05;
		digitalWrite(reservoir.ioPin, LOW); 
		digitalWrite(reservoir.pwmPin, LOW);
		
		pinMode(reservoir.ioPin, OUTPUT);
		pinMode(reservoir.pwmPin, OUTPUT); 
		
		/* Set the output range of the control algorithm.
		 +255 -> fill up at the fastest rate possible
		 0 -> seal the reservoir
		 -255-> bleed pressure from the reservoir at the fastest rate possible */
		algorithm.SetOutputLimits(-255, 255);
		algorithm.SetSampleTime(PIDUPDATECYCLE);
		algorithm.SetMode(AUTOMATIC);
		algorithm.SetTunings(reservoir.pTerm, reservoir.iTerm, reservoir.dTerm);
		
		nextUpdateTime = millis() + PIDUPDATECYCLE;
    }
	/*
		// CONSTRUCTOR basic
	PressureReservoir(ReservoirConfig newReservoir, PressureSensor newSensor, AquinasGlobal global)
	:  algorithm(	&real,	 // call the constructor for algorithim object
				 &controlOutput,
				 &setpoint, 
				 0 ,							// default values only - actual values set in constructor
				 0, 							
				 0, 							
				 DIRECT),
    sensor(newSensor),    	  	// call the constructor for sensor object
    controlOutput(0), 				// set Error Output to 0 (no flow in or out
    real(0),			// clear the current pressure
    setpoint(0),							// clear the setpoiint
    reservoir(newReservoir) {
		TCCR1B = TCCR1B & 0b11111000 | 0x05;	// Set frequency for pwmValve to ~32Hz
		digitalWrite(reservoir.ioPin, LOW);	
		digitalWrite(reservoir.pwmPin, LOW);
		
		pinMode(reservoir.ioPin, OUTPUT);
		pinMode(reservoir.pwmPin, OUTPUT); 
		
	 // Set the output range of the control algorithm.
	 // +255 -> fill up at the fastest rate possible
	 // 0 -> seal the reservoir
	 // -255-> bleed pressure from the reservoir at the fastest rate possible 
		algorithm.SetOutputLimits(-255, 255);
		algorithm.SetSampleTime(PIDUPDATECYCLE);
		algorithm.SetMode(AUTOMATIC);
		algorithm.SetTunings(reservoir.pTerm, reservoir.iTerm, reservoir.dTerm);
		
		nextUpdateTime = millis() + PIDUPDATECYCLE;
    }
	*/
	
	
	/**
	 * Updates the pressure reservoir controller if needed.
	 *
	 */
	void update()
	{
		if ( millis() > nextUpdateTime  )  // only update every 100msecs
		{
			readPressure(); 
			algorithm.Compute();
			setOutput();
			
			nextUpdateTime = nextUpdateTime + PIDUPDATECYCLE;
		} 
	}
	
	
private:
	AquinasGlobal *global;
	
	/**
	 * Takes the output from the ADC and scales it accordingly. 
	 *
	 * @param Pressure in counts (0-1023)
	 * @return Pressure in PSIG 
	 */
	double countsToPressure (double counts)
    {
		double voltageSpan = sensor.maxFSVolts - sensor.minFSVolts;
		double pressureSpan = sensor.maxPressure - sensor.minPressure;     
		
		double scaledPressure = sensor.minPressure + (pressureSpan * ( counts - sensor.minFSVolts ) / voltageSpan  );
		return scaledPressure;
		/*
		double volts = counts * 5.0 /1023.0;
		if (volts <= sensor.calibration.in[0]) return sensor.calibration.out[0];
		if (volts >= sensor.calibration.in[sensor.calibration.points-1])
            return sensor.calibration.out[sensor.calibration.points-1];
		
			// search right interval
		uint8_t pos = 1;  // _in[0] allready tested
		while(volts > sensor.calibration.in[pos]) pos++;
		
			// this will handle all exact "points" in the _in array
		if (volts == sensor.calibration.in[pos]) return sensor.calibration.out[pos];
		
			// interpolate in the right segment for the rest
		return map(volts, sensor.calibration.in[pos-1], sensor.calibration.in[pos], 
				   sensor.calibration.out[pos-1], sensor.calibration.out[pos]); 
		*/
	}
	
	/**
	 * Reads the ADC and sets current
	 *
	 */
	void readPressure()
	{
		rawSensor = 0;
			// take multiple readings of the pressure sensor and sum them all up
		for (int i=0; i < PRESSUREWINDOWWIDTH; i++)
			rawSensor += analogRead( sensor.pin );
		
			// scale and store pressure reading
		real = countsToPressure( (double)rawSensor / PRESSUREWINDOWWIDTH); 
		
	}
	
	
	/**
	 * Sets the positions of the hold and drive valves 
	 *
	 * @param Pressure in counts ADC reading (0-1023)
	 * @return Pressure in PSIG
	 */
	void setOutput()
	{
			// make sure we're set to 40Hz... might not need to check every loop?
		if (TCCR1B & 0b00000111 != 0x05)
			TCCR1B = TCCR1B & 0b11111000 | 0x05;
		
		/* When checking for direction, don't touch ioPin is CO = 0 */
		
			// if controlOutput is negative decrease reservoir pressure
		if (controlOutput < 0) digitalWrite(  reservoir.ioPin, HIGH);
		
			// if controlOutput is positive is increase reservoir pressure
		if  (controlOutput > 0) digitalWrite(  reservoir.ioPin, LOW); 
		
			// Set the duty cycle of the pressure hold pin to the calculated value
		analogWrite( reservoir.pwmPin, getOutput() );
		
	}
	
	int getOutput()  // this allows for a transfer function to correct the output signal for physical factors such as deadtime, etc.
	{
		int shapedOutput = 0;
		
		shapedOutput = 255 - abs(controlOutput);
		
			//  	if ( shapedOutput > 245 ) shapedOutput = 255;
			//  	if ( shapedOutput < 10) shapedOutput = 0;
		
		return shapedOutput;
	}
};

#endif







