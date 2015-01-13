///////////////////////////////////////////////////
// Hardware specific configuration
//   for Kernel Controller 
// 
// July 9, 2012
//

double tempTopSet=34;  // Culture Tub setpoint in degrees Celcius
double tempHumSet=38;  // Humidity Tub setpoint in degrees Celcius
double tempSubSet=33;  // Substage Tub setpoint in degrees Celcius
double humiditySet=94; // Relative Humidity setpoint in %RH

// const long BANDGAP_CALIBRATION = 1125300L;  ; // Default calibration value is 1.1V*1023 counts * 1000 mv/V= 1125300 counts*mV
const long int BANDGAP_CALIBRATION = 1125300L; // 5v == 4852mv. 

// scale_constant = internal1.1Ref * 1023 * 1000
// where
// internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() 

///////////////////////////////////////////////////
// Sets the max heater values here. 0-100% = 0-255
const float HUMIDITY_HEATER_MAX = 255.0;
const float HUMIDITY_SUBSTAGE_MAX = 255.0;
const float HUMIDITY_CULTURE_MAX = 255.0;

const double consKp=300, consKi=10, consKd=0; // Culture Heater
const double ssKp=300, ssKi=5, ssKd=0;  // Substage Heater 
const double humKp=300, humKi=10, humKd=0; // Humidity Heater - temperature feedback

const float NOPOWER = -500.0;

///////////////////////////////////////////////////
// Sets the clock and data pins for the
// Sensiron SH75 temperature and humidity sensor
const uint8_t dataPin  =  18;
const uint8_t clockPin =  19;

const uint8_t statusA =  12;
const uint8_t statusB =  13;
const uint8_t statusI2C =  3;
const uint8_t statusVcc = A6;

const uint8_t enableA =  7;
const uint8_t heaterA1 = 5;     // Humidity
const uint8_t heaterA1fb = A3;
const uint8_t heaterA2 = 6;     // Culture
const uint8_t heaterA2fb = A2;

const uint8_t enableB =  8;
const uint8_t heaterB1 = 9;
const uint8_t heaterB1fb = A1;
const uint8_t heaterB2 = 10;
const uint8_t heaterB2fb = A0;

const uint8_t valve1 = 2;
const uint8_t valve2 = 4;


