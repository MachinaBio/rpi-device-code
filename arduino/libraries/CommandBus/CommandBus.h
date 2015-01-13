/**
 * A hardware abstraction class for the valvestack communication bus
 *
 * This class handles valvestack bus communications and maintains an
 * internal register of the current output valve states.
 *
 *  Changes:
 *		2011 10 11 : Changed network speed to 115200
 *
 * @file CommandBus.H
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
#ifndef VSCOMMANDBUS
#define VSCOMMANDBUS

// **************************************************************************
// 8/5/2012, CJQ - Patched to compile with Arduino 1.0, 
//                            as well as older Arudino (e.g. 0023)
// **************************************************************************
#if (ARDUINO >= 100)
	#include <Arduino.h>
#else
	#include "WConstants.h"
#endif

class CommandBus {
public:
	/**
	 * Timeout for watchdog in milliseconds.
	 *
	 */
	String hVersion;
	String sVersion; 
	long currentPressure;
	bool recalibrate;
	bool newCal;
	float rawPressure;
	
	/**
	 * Command has been queued and is waiting for processing.
	 *
	 */
	boolean valid;
	
	boolean valveRegister[NUMVALVES];
	double setpointRegister;
	
	/**
	 * Constructor that initializes the network bus.
	 * 
	 * @param moduleAddress Single character address of this valve stack module
	 */
/*	CommandBus( String hVersion, String sVersion, int nValves) :
	currentPressure(0)
	{
		for (int i=0; i< MAXNVALVES; i++ ) valveRegister[i] = false;
	} 
*/	
	
	CommandBus(AquinasGlobal* address){
		global = address;
	}
	/**
	 * Performs startup tasks that can't be done in the constructors.
	 * Constructors are called in the global scope, but Serial.begin throws an
	 * error if called from the global scope.
	 */
	void begin()
	{
		Serial.begin(BUSSPEED);
		watchdogTimeout=millis()+NETWORKFAILSAFEDELAY;
        clearCommand();
        Serial.flush();
	}
	
	/**
	 * Performs periodic tasks required for bus activity.
	 *
	 */
	void update()
	{
		if (Serial.available() > 0 ) watchdogTimeout = millis() + NETWORKFAILSAFEDELAY;  
			// reset watchdog if there's any data waiting
		
			//		if ( (long)( millis() - watchdogTimeout) >= 0) (*global).failed=true;
			// set the global failed flag if watchdog expired
		
		while (Serial.available() > 0) {
			push(Serial.read());
			if ( _command == 'v' && valid ) updateValves();  // handle valve command internally
			if ( _command == 's' && valid ) updateSetpoint();  // handle setpoint command internally
			if ( valid ) break; 
		}
	}
	
	SensorCalibration getCalibration()
    {
		newCal = false;
		return calibration;
	}
	
private:
	AquinasGlobal *global;
	unsigned long watchdogTimeout;
	SensorCalibration calibration;
	
	int nValves;
	
	char _command;
	boolean commandExpected;
	int nBytesExpecting;
	
	/* Private stores for incoming command buffer */
	char payloadBuffer[MAXPAYLOADLENGTH];
	int index;
	
		// PUSH ::  Adds serialData to the stack as well as echoing it to upstream serial bus.
	void push(char serialData)
	{
		Serial.print(serialData); 
		/* Were we expecting anything in particular? */
		if (commandExpected) receiveCommand(serialData);	
		else if (nBytesExpecting > 0) receivePayload(serialData);
		else if (serialData == (*global).address)  {
			valid = false;
			commandExpected = true;
		}
	}
	
	void receivePayload(char serialData)
	{
		String validCharacters = String(VALIDPAYLOADCHARACTERS);
		
		if ( validCharacters.indexOf(serialData) == -1) {
			clearCommand();
			return; }
		
		payloadBuffer[index] = serialData;
		++index;
		if (--nBytesExpecting == 0) valid=true; // if that was the last byte, set valid  
	}
	
    void receiveCommand(char serialData)
	{
		String validCharacters = String(VALIDCOMMANDCHARACTERS);
		
		if ( validCharacters.indexOf(serialData) == -1) {
			clearCommand();
			return;
    	}
		_command = serialData;
		commandExpected = false;
		nBytesExpecting = MAXPAYLOADLENGTH; // we need to be able to vary the payload length...
	}
	
	/**
	 * Returns the current command from the bus, or "?" is there is no valid command.
	 *
	 * DEPRECATE
	 *
	 * @return The current command
	 */
	char getCommand()
	{
		if (valid == false) return '?';
		clearCommand();
		return _command;
	}
	
	/**
	 * Resets the command parser
	 *
	 */
	void clearCommand()
	{
		valid=false;
		nBytesExpecting=0;
		index=0;
	}
	
	/**
	 * Reads payload buffer and sets new setpoint
	 *
	 */
	void updateSetpoint()
	{
		setpointRegister = atof(payloadBuffer);
		clearCommand();
	}
	
	/**
	 * Reads payload buffer and sets valve states
	 *
	 */
	void updateValves()
	{
		for(int i=0; i < nValves;  i++)
		{
			valveRegister[i] = false;
			if ( payloadBuffer[i] == '1' ) valveRegister[i] = true; }
		clearCommand();
	}
};

#endif
















