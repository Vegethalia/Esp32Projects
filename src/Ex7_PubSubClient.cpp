#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "modules/MyBME280.h"
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#define ADAFRUIT_ADDR "io.adafruit.com"
#define ADAFRUIT_PORT 1883

// ADAFRUIT Feeds
#define FEED_TEMPERATURE "/feeds/temperatura"
#define FEED_HUMIDITY    "/feeds/humitat"
#define FEED_PRESSURE    "/feeds/pressio"
// //AdafruitIO_Feed *adafruit_alt = iowifi.feed("Altitut");
// AdafruitIO_Feed *adafruitread_blueledon = iowifi.feed(FEED_TURN_ON_PWN);
// AdafruitIO_Feed *adafruitread_blueledpower = iowifi.feed(FEED_INTENSITY_PWN);
#define PRESSURE_OFFSET 14 //looks like my sensor always returns the real pressure minus this offset

#define PIN_I2C_SDA        21
#define PIN_I2C_SCL        22
#define PIN_LED            32
#define PIN_BUTTON         35
#define I2C_ADDRESS_BME280 0x76
#define I2C_BUS_SPEED      100000 //100000, 400000

//GLOBAL OBJECTS
WiFiClient _TheWifi;
PubSubClient _ThePubSub;
TwoWire _TheIc2Wire(0);
//Adafruit_BME280 _TheBME; // BME280 I2C
MyBME280 _TheBME;

//TIMING VARS
unsigned long _delayTimeUpt=30000;
unsigned long _delayTimeLoop=50;
unsigned long _lastProcessMillis=0;

void setup()
{
 	Serial.begin(115200);
  // wait for serial monitor to open
  while(!Serial);

	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_BUTTON, INPUT);

	//Initialize I2C bus
	log_d("Initializing i2c sda=%d scl=%d speed=%dkhz", PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_SPEED/1000);
	if(!_TheIc2Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_SPEED)) { //busspeed=0 => default 100000 
		log_d("i2c initialization failed...");
	}
	//Initialize BME280
	_TheBME.Init(I2C_ADDRESS_BME280, _TheIc2Wire, 20000);
	_TheBME.SetPressureOffset(PRESSURE_OFFSET);

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
				if(!_ThePubSub.connect("PChanMQTT", ADAIO_USER, ADAIO_KEY)) {
					log_d("ERROR!! PubSubClient was not able to connect to AdafruitIO!!");
					rdy=false;
				}
			}
		}
		if(rdy) {
			if(_TheBME.ReadSensor()==MyBME280::ERROR::OK) {
				_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_TEMPERATURE)).c_str(), String(_TheBME.GetLatestTemperature()).c_str());
				_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_HUMIDITY)).c_str(), String(_TheBME.GetLatestHumidity()).c_str());
				_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_PRESSURE)).c_str(), String(_TheBME.GetLatestPressure()).c_str());
			}
		}
	}

	delay(_delayTimeLoop);	
}
