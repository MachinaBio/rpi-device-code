#ifndef AQUINAS
#define AQUINAS 
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
 
 /*
 NOTES:
 1- Millis() rollover handled by 2's complement math workaround. On http://www.arduino.cc/playground/Code/TimingRollover
        Example:    (nextTime=millis() + 1000;
                    if ( (long)( millis() - nextUpdateTime) >= 0 )the time elapsed!
 
 */
 
#define NUMVALVES 8
#define PIDUPDATECYCLE 250       // Cycle t 
#define PRESSUREWINDOWWIDTH 8    // Number of pressure samples to average for each reading
#define DISPLAYUPDATETIME 500    // Update the screen every 500ms, ie. 2Hz

/* Set the LCD backlight brightness level, value between 1 to 8 This command set the LCD display backlight brightness level, the value is between 1 to 8. Default brightness value is 1.*/
#define BRIGHTBACKLIGHT 8
#define DIMBACKLIGHT 6	


/* Set the display contrast, value between 1 to 50 This command sets the LCD character display contrast, the contrast setting is between 1 to 50, where 50 is the highest contrast. Default contrast value is 40. */
#define BRIGHTCONTRAST 50
#define DIMCONTRAST 50

#define BUSSPEED 38400
#define NETWORKFAILSAFEDELAY 5000
#define MAXPAYLOADLENGTH 8
#define MAXNVALVES 8

#define VALIDINTERRUPTCHARACTERS "?!"
#define VALIDCOMMANDCHARACTERS "vsq"
#define VALIDPAYLOADCHARACTERS "1234567890."
 
struct ReservoirConfig {
    int     ioPin;   // control direction valve (H=fill, L=empty))
    int     pwmPin;    // controls duty cycle of flow valve (H=change, L=block)
    int     cycle;      // cycle time in ms of PID loop
    int     nValves;    // number of output valves
    double  pTerm;
    double  iTerm;
    double  dTerm;
};

struct SensorCalibration {
    double  in[5];  
    double  out[5];
    int     points;  // number of valid calibration points
};

struct  LCDconfig {
    uint8_t rows;
    uint8_t columns;
    uint8_t i2cAddress;
};

struct PressureSensor {
  int    pin;
  double minPressure;  // these are fallbacks for when no calibration availabile.
  double maxPressure;
  double minFSVolts;
  double maxFSVolts;
  SensorCalibration calibration;
};

struct AquinasGlobal {
  double kPa_reservoir;
  double kPa_setpoint;
    bool valves[NUMVALVES];
	char address;
	bool failed;
	bool failsafe[NUMVALVES];
	long rawCounts;
  String hVersion;
  String sVersion;
  SensorCalibration calibration;
};
#endif
