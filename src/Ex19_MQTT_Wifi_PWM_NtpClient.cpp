
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <string>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

//PINS & ADDRESSES...
#define PIN_I2C_SDA        21
#define PIN_I2C_SCL        22
#define PIN_LED_PWM        32

#define PWMCHANNEL    0
#define PWMRESOLUTION 8

// Feeds
#define MQTT_BROKER      "192.168.1.140"
#define MQTT_PORT        1888

#define TOPIC_INTENSITY  "caseta/leds/intensity"
#define TOPIC_STYLE      "caseta/leds/style"
#define TOPIC_ALWAYS_ON  "caseta/leds/alwayson"
#define TOPIC_TIME       "caseta/leds/time"

#define UPDATE_TIME_EVERY_SECS 10

//GLOBAL OBJECTS
WiFiClient    _TheWifi;
PubSubClient  _ThePubSub;
WiFiUDP       _TheWifi4UDP;
NTPClient     _TheNTPClient(_TheWifi4UDP);

//TIMING VARS
uint32_t _lastTimeSent=0;

//FORWARD DECLARATIONS
void PubSubCallback(char *pTopic, uint8_t *pData, unsigned int dalaLength);

//state vars
uint32_t _Intensity=100;
bool     _ledsOn=false;

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while(!Serial);

	pinMode(PIN_LED_PWM, OUTPUT);

	ledcSetup(PWMCHANNEL, 5000, PWMRESOLUTION); //channel0, freq=¿1000?, resolution=8bits
	ledcAttachPin(PIN_LED_PWM, PWMCHANNEL);

	_TheNTPClient.setTimeOffset(3600);
	//Initialize Wifi
	wl_status_t statuswf = WiFi.begin(WIFI_SSID, WIFI_PASS);
	statuswf = (wl_status_t)WiFi.waitForConnectResult();
	if ( statuswf != WL_CONNECTED) {
		log_d("Couldn't get a wifi connection!");
	}
	else {
		Serial.println("Connected to wifi");
		_TheNTPClient.begin();
	}
}

void loop()
{
	bool rdy=true;

	auto now=millis();

	if(!WiFi.isConnected()) {
		log_d("Reconnecting to Wifi...");
		WiFi.reconnect();
	}
	if(WiFi.status()!= WL_CONNECTED) {
		log_d("Couldn't get a wifi connection!");
		rdy=false;
	}
	else {
		_TheNTPClient.update();

		if(!_ThePubSub.connected()) {
			_ThePubSub.setClient(_TheWifi);
			_ThePubSub.setServer(MQTT_BROKER, MQTT_PORT);
			_ThePubSub.setCallback(PubSubCallback);
			if(!_ThePubSub.connect("ESP32_test")) {
				log_d("ERROR!! PubSubClient was not able to connect to PiRuter!!");
				rdy=false;
			}
			else { //Subscribe to the feeds
				log_d("PubSubClient connected to PiRuter MQTT broker!!");
				if(!_ThePubSub.subscribe(TOPIC_INTENSITY)) {
					log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_INTENSITY);
				}
				if(!_ThePubSub.subscribe(TOPIC_ALWAYS_ON)) {
					log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_ALWAYS_ON);
				}
			}
		}
	}
	if((now - _lastTimeSent) > UPDATE_TIME_EVERY_SECS*1000) {
			_ThePubSub.publish(TOPIC_TIME, _TheNTPClient.getFormattedTime().c_str(), true);
			_lastTimeSent=now;
	}

	if(_ThePubSub.connected()) {
		_ThePubSub.loop(); //allow the pubsubclient to process incoming messages
	}
	// delay(250);
	// digitalWrite(PIN_LED, HIGH);
	// delay(250);
	// digitalWrite(PIN_LED, LOW);
	if(!WiFi.isConnected()) {
		delay(1000);
	}
}

void PubSubCallback(char *pTopic, uint8_t *pData, unsigned int dataLenght)
{
	std::string theTopic(pTopic);
	std::string theMsg;

	for(uint16_t i=0; i<dataLenght; i++) {
		theMsg.push_back((char)pData[i]);
	}
	log_d("Received message from [%s]: [%s]", theTopic.c_str(), theMsg.c_str());

	if(theTopic.find(TOPIC_ALWAYS_ON)!=std::string::npos) {
		if(theMsg=="SI") {
			log_d("Turning on the LEDS!, intensity=%d", _Intensity);
			ledcWrite(PWMCHANNEL, _Intensity);
			_ledsOn = true;
		}
		else {
			log_d("Turning off the LEDS...");
			ledcWrite(PWMCHANNEL, 0);
			_ledsOn = false;
		}
	}
	else if(theTopic.find(TOPIC_INTENSITY)!=std::string::npos) {
		auto intensity=std::atoi(theMsg.c_str());
		log_d("Changing led intensity=%d", intensity);
		_Intensity=intensity;
		if(_ledsOn) {
			ledcWrite(PWMCHANNEL, _Intensity);
		}
	}
}
