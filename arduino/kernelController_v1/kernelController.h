///////////////////////////////////////////////////
// Hardware specific configuration
//   for Kernel Controller 
// 
// July 9, 2012
//

///////////////////////////////////////////////////
// Sets the max heater values here. 0-100% = 0-255
const float HUMIDITY_HEATER_MAX = 50.0;
const float HUMIDITY_SUBSTAGE_MAX = 255.0;
const float HUMIDITY_CULTURE_MAX = 34.8;
const float NOPOWER = -50;

#define Hum_HtrResponse(x) (fscale(-50.0, 49.0, 0.0, HUMIDITY_HEATER_MAX, x, 0))
#define Sub_HtrResponse(x) (fscale(-50.0, 49.0, 0.0, HUMIDITY_SUBSTAGE_MAX, x, 0))
#define Top_HtrResponse(x) (fscale(-50.0, 49.0, 0.0, HUMIDITY_CULTURE_MAX, x, 0))

///////////////////////////////////////////////////
// Sets the clock and data pins for the
// Sensiron SH75 temperature and humidity sensor
const uint8_t dataPin  =  18;
const uint8_t clockPin =  19;

const uint8_t statusA =  12;
const uint8_t statusB =  13;
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

// Culture Heater :: Define the aggressive and conservative Tuning Parameters 
const double aggKp=170, aggKi=0, aggKd=0; // AGGRESSIVE: proportional band +/- 2.0C 
const double consKp=50, consKi=.25, consKd=0; // CONSERVATIVE: proportional band +/- 8

// Substage Heater 
const double ssKp=10, ssKi=0.1, ssKd=0;

// Humidity Heater 
const double humKp=1, humKi=0.05, humKd=0;

