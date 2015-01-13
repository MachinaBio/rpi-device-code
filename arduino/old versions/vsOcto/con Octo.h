///////////////////////////////////////////////////
// Hardware specific configuration
///////////////////////////////////////////////////
/* Sets address of this module */
const char moduleAddress = 'A';

/* Sets address of this module */
const double octoSetpoint = 15;

// #define PSI36 1
#define PSI72 1

/* Set the pins for each of the valves */
//const int octoVPins[8] =  {9, 8, 7, 6, 5, 4, 3, 2};
const int octoVPins[8] =  {2, 4, 6, 8, 3, 5, 7, 9};
const int octoNValves =    8;

// Bus speed is set in CommandBus.h
/// ALSO CHANGE SERIAL FUNCTION IF USING OTHER THAN D0 = RECIEVE and D1 = TRANSMIT

////////////////////////////////////////////////
//  PARAMETERS FOR PRESSURE SENSOR
////////////////////////////////////////////////

/* The default values for the reservoir PID object */
const ReservoirConfig octoReservoirConfig = {   11, // Pin for the bleed/fill valve 
                                                10, // Pin for the hold valve 
                                              33.0, // Proportional term
                                                50, // Integral term
                                               0.0 };  // Derivative term
                                               
////////////////////////////////////////////////
//  PARAMETERS FOR PRESSURE SENSOR
////////////////////////////////////////////////

#ifdef PSI72     
// for 72.5 psi sensor
const PressureSensor octoPresureSensor = { A0, // Analog in pin for pressure sensor 
                                         (double) 0.0, // Lower limit of sensor pressure range 
                                         (double) 72.5, // Upper limit of sensor pressure range 
                                        (double) 0.20, // Lower limit of sensor voltage output
                                        (double) 4.70 }; // Upper limit of sensor voltage output
#endif    

// for 36 psi sensor
#ifdef PSI36
const PressureSensor octoPresureSensor = { A0, // Analog in pin for pressure sensor 
                                         (double) 0.0, // Lower limit of sensor pressure range 
                                         (double) 36.3, // Upper limit of sensor pressure range 
                                        (double) 0.205, // Lower limit of sensor voltage output
                                        (double) 4.705 }; // Upper limit of sensor voltage output
#endif
                      
const LCDconfig octoLCD = {2,   // number of rows
                          16,   // number of columns
                        0x50};  // I2C address
