/*
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string>
#include <Adafruit_TSL2591.h>
#include "modules/MyBME280.h"
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

//PINS & ADDRESSES...
#define PIN_I2C_SDA        21
#define PIN_I2C_SCL        22
#define PIN_LED            32
#define PIN_LED_PWM        25
#define I2C_ADDRESS_BME280 0x76
#define I2C_BUS_SPEED      100000 //100000, 400000

#define PWMCHANNEL         0
#define PWMRESOLUTION      8

// ADAFRUIT Feeds / Related stuff
#define ADAFRUIT_ADDR      "io.adafruit.com"
#define ADAFRUIT_PORT      1883

#define FEED_TEMPERATURE   "/feeds/temperatura"
#define FEED_HUMIDITY      "/feeds/humitat"
#define FEED_PRESSURE      "/feeds/pressio"
#define FEED_TURN_ON_PWM   "/feeds/turnonled"
#define FEED_INTENSITY_PWN "/feeds/ledintensity"
#define FEED_LUX           "/feeds/lux"

#define PRESSURE_OFFSET    14    //looks like my sensor always returns the real pressure minus this offset
#define LUX_TMIN_GAIN      5000  //Minimum Lux threshold to decrease timing/gain
#define LUX_TMAX_GAIN      20000 //Maximum Lux threshold to decrease timing/gain

//GLOBAL OBJECTS
WiFiClient    _TheWifi;
PubSubClient  _ThePubSub;
TwoWire       _TheIc2Wire(0);
MyBME280      _TheBME;
Adafruit_TSL2591 _TheTSL = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

//TIMING VARS
unsigned long _delayTimeUpt=30000;
unsigned long _delayTimeLoop=50;
unsigned long _lastProcessMillis=0;

//Adafruit_TSL2591
tsl2591Gain_t _lastTslGain=TSL2591_GAIN_MED;
tsl2591IntegrationTime_t _lastTslIntegrationTime=TSL2591_INTEGRATIONTIME_300MS;

//FORWARD DECLARATIONS
void PubSubCallback(char *pTopic, uint8_t *pData, unsigned int dalaLength);
float AdvancedTSL2591Read();

//state vars
bool _BlueLedON=false;
byte _BlueLedIntensity=64; //1/4 intensity at 8 bits

void setup()
{
 	Serial.begin(115200);
  // wait for serial monitor to open
  while(!Serial);

	pinMode(PIN_LED, OUTPUT);
	//Configure PWM pin
	ledcSetup(PWMCHANNEL, 10000, PWMRESOLUTION); //channel0, freq=¿10000?, resolution=8bits
	ledcAttachPin(PIN_LED_PWM, PWMCHANNEL);

	//Initialize I2C bus
	log_d("Initializing i2c sda=%d scl=%d speed=%dkhz", PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_SPEED/1000);
	if(!_TheIc2Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_SPEED)) { //busspeed=0 => default 100000 
		log_d("i2c initialization failed...");
	}
	//Initialize BME280
	_TheBME.Init(I2C_ADDRESS_BME280, _TheIc2Wire, 20000);
	_TheBME.SetPressureOffset(PRESSURE_OFFSET);

	//Initialize the TSL2591
	if(_TheTSL.begin())  {
    log_d("Found a TSL2591 sensor");
		_TheTSL.setGain(_lastTslGain); // 25x gain
		_TheTSL.setTiming(_lastTslIntegrationTime);
  }
  else {
    log_d("No sensor found ... check your wiring?");
  }

	//Initialize Wifi
	wl_status_t statuswf = WiFi.begin(WIFI_SSID, WIFI_PASS);
  if ( statuswf != WL_CONNECTED) {
    log_d("Couldn't get a wifi connection!");
  }
  else {
    Serial.println("Connected to wifi");
	}
}

void loop()
{
	bool rdy=true;

	auto now=millis();

	if((now-_lastProcessMillis)>=_delayTimeUpt) {
		_lastProcessMillis=now;

		digitalWrite(PIN_LED, HIGH);	// turn on the LED
		delay(200);	
		digitalWrite(PIN_LED, LOW);	// turn off the LED

		if(!WiFi.isConnected()) {
			log_d("Reconnecting to Wifi...");
			WiFi.reconnect();
		}
		if(WiFi.status()!= WL_CONNECTED) {
			log_d("Couldn't get a wifi connection!");
			rdy=false;
		}
		else {
			if(!_ThePubSub.connected()) {
				_ThePubSub.setClient(_TheWifi);
				_ThePubSub.setServer(ADAFRUIT_ADDR, ADAFRUIT_PORT);
				_ThePubSub.setCallback(PubSubCallback);
				if(!_ThePubSub.connect("PChanMQTT", ADAIO_USER, ADAIO_KEY)) {
					log_d("ERROR!! PubSubClient was not able to connect to AdafruitIO!!");
					rdy=false;
				}
				else { //Subscribe to the on/off button and the slider
					log_d("PubSubClient connected to AdafruitIO!!");
					if(!_ThePubSub.subscribe((std::string(ADAIO_USER).append(FEED_TURN_ON_PWM)).c_str())) {
						log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", FEED_TURN_ON_PWM);
					}
					if(!_ThePubSub.subscribe((std::string(ADAIO_USER).append(FEED_INTENSITY_PWN)).c_str())) {
						log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", FEED_INTENSITY_PWN);
					}
				}
			}
		}
		if(rdy) {
			if(_TheBME.ReadSensor()==MyBME280::ERROR::OK) {
				_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_TEMPERATURE)).c_str(), String(_TheBME.GetLatestTemperature()).c_str());
				_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_HUMIDITY)).c_str(), String(_TheBME.GetLatestHumidity()).c_str());
				_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_PRESSURE)).c_str(), String(_TheBME.GetLatestPressure()).c_str());
			}
			auto lux = AdvancedTSL2591Read();
			if(lux>=0) {
				_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_LUX)).c_str(), String(lux).c_str());
			}
		}
	}

	if(_ThePubSub.connected()) {
		_ThePubSub.loop(); //allow the pubsubclient to process incoming messages
	}
	delay(_delayTimeLoop);	
}

void PubSubCallback(char *pTopic, uint8_t *pData, unsigned int dataLenght)
{
	std::string theTopic(pTopic);
	std::string theMsg;

	for(uint16_t i=0; i<dataLenght; i++) {
		theMsg.push_back((char)pData[i]);
	}
	log_d("Received message from [%s]: [%s]", theTopic.c_str(), theMsg.c_str());

	if(theTopic.find(FEED_TURN_ON_PWM)!=std::string::npos) {
		if(theMsg=="ON") {
			log_d("Turning on the blue LED!, intensity=%d", _BlueLedIntensity); 
			_BlueLedON=true;
			ledcWrite(PWMCHANNEL, _BlueLedIntensity);
		}
		else {
			log_d("Turning off the blue led...");
			_BlueLedON=false;
			ledcWrite(PWMCHANNEL, 0);
		}
	}
	else if(theTopic.find(FEED_INTENSITY_PWN)!=std::string::npos) {
		auto intensity=std::atoi(theMsg.c_str());
		log_d("Changing led intensity=%d", intensity); 
		_BlueLedIntensity=intensity;
		if(_BlueLedON) {
			ledcWrite(PWMCHANNEL, _BlueLedIntensity);
		}
	}
}
float AdvancedTSL2591Read(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = _TheTSL.getFullLuminosity();
  uint16_t ir, full, visible;
  ir = lum >> 16;
  full = lum & 0xFFFF;
	visible = full - ir;

	float lux = _TheTSL.calculateLux(full, ir);
	bool changeGain=false;

	log_d("TSL2591 --> Last Visible=%d IR=%d LUX=%2.1f", visible, ir, lux);

	//auto adjust gain/timmin based on thresholds
	if(lux<0) { //overflow!!
		if (_lastTslGain > tsl2591Gain_t::TSL2591_GAIN_LOW) {
			_lastTslGain = (tsl2591Gain_t)(_lastTslGain - 0x10);
		}
		//_lastTslIntegrationTime = tsl2591IntegrationTime_t::TSL2591_INTEGRATIONTIME_300MS;
		changeGain = true;
		log_d("TSL2591 --> OVERFLOW!! Decreasing gain to %d, time to %dms", _lastTslGain, (_lastTslIntegrationTime + 1) * 100);
	}
	else if (visible*10 < LUX_TMIN_GAIN)	{ //low values, increase gain!
		if(_lastTslGain < tsl2591Gain_t::TSL2591_GAIN_HIGH) { //increase gain
			_lastTslGain = (tsl2591Gain_t)(_lastTslGain + 0x10);
			changeGain = true;
		}
	}
	else if (visible*10 > LUX_TMAX_GAIN)	{ //high values, decrease gain!
		if (_lastTslGain > tsl2591Gain_t::TSL2591_GAIN_LOW) {
			_lastTslGain = (tsl2591Gain_t)(_lastTslGain - 0x10);
			changeGain = true;
		}
	}
	if(changeGain) {
		_TheTSL.setGain(_lastTslGain);
		_TheTSL.setTiming(_lastTslIntegrationTime);
		log_d("TSL2591 --> Changed GAIN to %d -- Integration Time  to %dms", _lastTslGain, (_lastTslIntegrationTime + 1) * 100);
	}
	return lux;
}
*/