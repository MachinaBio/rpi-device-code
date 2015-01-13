///////////////////////////////////////////////////
// Hardware specific configuration
///////////////////////////////////////////////////
/* Sets address of this module */
const char moduleAddress = 'A';
char moduleName[7] = "vsOcto"; // make sure the length is 1 longer than the number of characters in the name

/* Sets address of this module */
const double octoSetpoint = 15;

/* Set the pins for each of the valves */
const int octoVPins[8] =  {9, 8, 7, 6, 5, 4, 3, 2};
const int octoNValves =    8;

// const int vsBusFreq = 9600;
/// ALSO CHANGE SERIAL FUNCTION IF USING OTHER THAN D0 = RECIEVE and D1 = TRANSMIT

/* The default values for the reservoir PID object */
const ReservoirConfig octoReservoirConfig = {   10, // Pin for the reservoir hold valve 
                                                11, // Pin for the bleed/fill valve 
                                             200.0, // Proportional term
                                               2.0, // Integral term
                                                0.0 };  // Derivative term

const PressureSensor octoPresureSensor = {  8, // Analog in pin for pressure sensor 
                                            0, // Lower limit of sensor pressure range 
                                         36.3, // Upper limit of sensor pressure range 
                                        0.220, // Lower limit of sensor voltage output
                                        4.705 }; // Upper limit of sensor voltage output
                                  

