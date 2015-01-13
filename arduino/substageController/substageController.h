///////////////////////////////////////////////////
// Hardware specific configuration
//   for bioChamber v1
///////////////////////////////////////////////////

// Used for LCDi2CNHD library
#define VERSION "1.1"

// Sets the clock and data pins for the
// Sensiron SH75 temperature and humidity sensor
const uint8_t chamberDataPin  =  8;
const uint8_t chamberClockPin =  10;

const uint8_t chamberDetectPin  =  9;
const uint8_t chamberHeaterPin =  6;

const uint8_t substageDataPin  =  11;
const uint8_t substageClockPin =  13;

const uint8_t substageDetectPin  =  12;
const uint8_t substageHeaterPin =  7;

//Define the aggressive and conservative parameters for chamber heating
double aggKp=170, aggKi=0, aggKd=0; // AGGRESSIVE: proportional band +/- 2.0C 
double consKp=70, consKi=0.25, consKd=0; // CONSERVATIVE: proportional band +/- 8

// Define substage heating parameters
double ssKp=25, ssKi=0.5, ssKd=0; // CONSERVATIVE: proportional band +/- 2
