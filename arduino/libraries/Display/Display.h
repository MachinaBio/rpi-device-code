/**
 * A hardware abstraction class for the LCD display
 *
 * This class handles periodic updating of on-screen information
 * and initializaiton of an NHD i2C LCD
 *
 * @file Display.H
 * @author Carlo Quinonez <cquin donez@ucsd.edu>
 * @version 1.0
 * @section LICENSE
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef VSDISPLAY
#define VSDISPLAY

#ifndef LCDi2cNHD_h
#error The LCDi2cNHD library must be included from your sketch.
#endif

// **************************************************************************
// 8/5/2012, CJQ - Patched to compile with Arduino 1.0, 
//                            as well as older Arudino (e.g. 0023)
// **************************************************************************
#if (ARDUINO >= 100)
	#include <Arduino.h>
#else
	#include "WConstants.h"
#endif

class AquinasDisplay {	 
public:
	
	AquinasDisplay( LCDconfig config , AquinasGlobal* address) :
	lcd(config.rows, config.columns, config.i2cAddress >> 1, 0),
	alarm(false)
	{
		nextUpdateTime=millis();
		global = address;
	}
	
		// Must be called in init()  
	void begin() {
		lcd.init();
		lcd.clear();
		lcd.setDelay(0,0);
		lcd.blink_off(); 
		lcd.cursor_off();
		setDim();
		loadInvertedDigits();
		lcd.clear();
		splash();
		delay(2000);
		lcd.clear();
	}
	
		// Updates the screen.
	void update() {
		if ( (long)( millis() - nextUpdateTime) >= 0) {
			nextUpdateTime = nextUpdateTime + DISPLAYUPDATETIME;
			lcd.clear();
			drawValves();
			drawPressures();
			drawAddress();
			checkAlarm();
			if (strobe) drawStrobe();
		}
	}
	
		///////////////////////////////////////////////////
		///////////////////////////////////////////////////
private:
	AquinasGlobal *global;
	LCDi2cNHD lcd;
	unsigned long nextUpdateTime;
	bool alarm;
	bool strobe;
	
	/* DRAWPRESSURE :: Draws contents of the pressure registers */
	void drawPressures () {
		char buffer[5];
		lcd.setCursor(1,0);
			//lcd.print("S ");
			//dtostrf( global.kPa_setpoint, 5, 2, buffer);
		dtostrf( (*global).kPa_setpoint, 5, 3, buffer);
		lcd.print(buffer);
		lcd.print(" kPa");
		
		lcd.setCursor(0,0);
			//		lcd.print("R ");
			//dtostrf( global.kPa_reservoir, 5, 2, buffer);
		dtostrf( (*global).kPa_reservoir, 5, 3, buffer);
		lcd.print(buffer);
		lcd.print(" kPa");
	}
	
	/* DRAWVALVES :: Draws contents of the valve register on the screen */
	void drawValves () {
			//	lcd.setCursor(1,8);
			//		for( int i=0; i < 8; i++) global.valves[i] ? lcd.print(i+1) : lcd.print('x') ;
		lcd.setCursor(0,12);
		for( int i=0; i < 4; i++) (*global).valves[i] ? lcd.print(i+1) : lcd.print('x') ;
		lcd.setCursor(1,12);
		for( int i=4; i < 8; i++) (*global).valves[i] ? lcd.print(i+1) : lcd.print('x') ;
		
	}
	
	/* DRAWADDRESS :: Draw the address register on the screen */
	void drawAddress ()
	{		
		char buffer = (*global).address;
		lcd.setCursor(0,10);
		lcd.print(buffer);
	}
	
	/* DRAWSTROBE :: Draw the address register on the screen */
	void drawStrobe ()
	{		
		lcd.setCursor(0,10);
		lcd.print('?');
	}
	
		// CHECK ALARM :: looks for trouble flag */
	void checkAlarm()
	{
		/* Check list of events... */
			// Network failed?
		if ((*global).failed) alarm=true;
		else alarm=false;
		
		/* If alarm is set, invert strobe */
		if (alarm) strobe = !strobe;
		else strobe = false;
	}
	
	/* SPLASH :: Draws the splash screen */
	void splash() {
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Aquinas Control");
		lcd.setCursor(1,0);
		lcd.print("Designed at UCSD");
	}
	
	
	void setBright() {
		lcd.setBacklight(BRIGHTBACKLIGHT);
		lcd.setContrast(BRIGHTCONTRAST); 
	}
	
		// SETDIM :: Sets the dim font
	void setDim() {
		lcd.setBacklight(DIMBACKLIGHT);
		lcd.setContrast(DIMCONTRAST);
	}
	
	void loadInvertedDigits()
	{
		byte customCharacter[8];
		
			// Create and save an inverted 1
		customCharacter[0] = B11011;
		customCharacter[1] = B10011;
		customCharacter[2] = B11011;
		customCharacter[3] = B11011;
		customCharacter[4] = B11011;
		customCharacter[5] = B11011;
		customCharacter[6] = B10001;
		customCharacter[7] = B11111;
		
		lcd.load_custom_character(0, customCharacter);
		
			// Create and save an inverted 2
		customCharacter[0] = B10001;
		customCharacter[1] = B01110;
		customCharacter[2] = B11110;
		customCharacter[3] = B11101;
		customCharacter[4] = B11011;
		customCharacter[5] = B10111;
		customCharacter[6] = B00000;
		customCharacter[7] = B11111;
		lcd.load_custom_character(1, customCharacter);
		
			// Create and save an inverted 3
		customCharacter[0] = B00000;
		customCharacter[1] = B11101;
		customCharacter[2] = B11011;
		customCharacter[3] = B11101;
		customCharacter[4] = B11110;
		customCharacter[5] = B01110;
		customCharacter[6] = B10001;
		customCharacter[7] = B11111;
		lcd.load_custom_character(2, customCharacter);
		
			// Create and save an inverted 4
		customCharacter[0] = B11101;
		customCharacter[1] = B11001;
		customCharacter[2] = B10101;
		customCharacter[3] = B01101;
		customCharacter[4] = B00000;
		customCharacter[5] = B11101;
		customCharacter[6] = B11101;
		customCharacter[7] = B11111;
		
		lcd.load_custom_character(3, customCharacter);
			// Create and save an inverted 5
		customCharacter[0] = B00000;
		customCharacter[1] = B01111;
		customCharacter[2] = B00001;
		customCharacter[3] = B11110;
		customCharacter[4] = B11110;
		customCharacter[5] = B01110;
		customCharacter[6] = B10001;
		customCharacter[7] = B11111;
		lcd.load_custom_character(4, customCharacter);
		
			// Create and save an inverted 6
		customCharacter[0] = B11001;
		customCharacter[1] = B10111;
		customCharacter[2] = B01111;
		customCharacter[3] = B00001;
		customCharacter[4] = B01110;
		customCharacter[5] = B01110;
		customCharacter[6] = B10001;
		customCharacter[7] = B11111;
		lcd.load_custom_character(5, customCharacter);
		
			// Create and save an inverted 7
		customCharacter[0] = B00000;
		customCharacter[1] = B11110;
		customCharacter[2] = B11101;
		customCharacter[3] = B11101;
		customCharacter[4] = B11011;
		customCharacter[5] = B11011;
		customCharacter[6] = B11011;
		customCharacter[7] = B11111;
		lcd.load_custom_character(6, customCharacter);
		
			// Create and save an inverted 8
		customCharacter[0] = B10001;
		customCharacter[1] = B01110;
		customCharacter[2] = B01110;
		customCharacter[3] = B10001;
		customCharacter[4] = B01110;
		customCharacter[5] = B01110;
		customCharacter[6] = B10001;
		customCharacter[7] = B11111;
		lcd.load_custom_character(7, customCharacter);
	}
};

#endif



































