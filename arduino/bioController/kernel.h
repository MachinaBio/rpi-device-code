///////////////////////////////////////////////////
// Hardware specific configuration
//   for Kernel Controller 
// 
// July 9, 2012
//
///////////////////////////////////////////////////

// Used for LCDi2CNHD library
#define VERSION "1.1"

// Sets the clock and data pins for the
// Sensiron SH75 temperature and humidity sensor
const uint8_t dataPin  =  18;
const uint8_t clockPin =  19;

const uint8_t statusA =  9;
const uint8_t statusB =  6;
const uint8_t statusI2C =  3;

const uint8_t enableA =  7;
const uint8_t heaterA1 = 5;
const uint8_t heaterA1fb = 2;
const uint8_t heaterA2 = 6;
const uint8_t heaterA2fb = 3;

const uint8_t enableB =  8;
const uint8_t heaterB1 = 9;
const uint8_t heaterB1fb = 1;
const uint8_t heaterB2 = 10;
const uint8_t heaterB2fb = 0;

const uint8_t valve1 = 2;
const uint8_t valve2 = 4;

//Define the aggressive and conservative Tuning Parameters
double aggKp=170, aggKi=0, aggKd=0; // AGGRESSIVE: proportional band +/- 2.0C 
double consKp=70, consKi=.25, consKd=0; // CONSERVATIVE: proportional band +/- 8

