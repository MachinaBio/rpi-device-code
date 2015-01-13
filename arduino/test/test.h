///////////////////////////////////////////////////
// Hardware specific configuration
//   for bioChamber v1
///////////////////////////////////////////////////

// Used for LCDi2CNHD library
#define VERSION "1.1"

// Sets the clock and data pins for the
// Sensiron SH75 temperature and humidity sensor
const uint8_t dataPin  =  11;
const uint8_t clockPin =  13;

const uint8_t stageDetectPin  =  12;
const uint8_t heaterPin =  7;

//Define the aggressive and conservative Tuning Parameters
double aggKp=170, aggKi=0, aggKd=0; // AGGRESSIVE: proportional band +/- 2.0C 
double consKp=70, consKi=.25, consKd=0; // CONSERVATIVE: proportional band +/- 8

