// Import required libraries
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "mykeys.h"

#define PIN_BAT_VOLTAGE 32
#define PIN_RELAY       26

#define MAX_VOLTAGE     4.25f
#define MIN_VOLTAGE     3.45f

#define RELAY_NO        false // Set to true to define Relay as Normally Open (NO)

#define MQTT_BROKER      "192.168.1.140"
#define MQTT_PORT        1888
#define TOPIC_VOLTAGE    "/caseta/remoteplug/battery"
#define TOPIC_SWITCH     "/caseta/remoteplug/switch"

#define SLEEP_TIME_SECS 60
#define DELAY_LOOP_MS   1500
#define MAX_RETRIES     10
#define uS_TO_S_FACTOR  1000000  //Conversion factor for micro seconds to seconds

//GLOBAL OBJECTS
WiFiClient    _TheWifi;
PubSubClient  _ThePubSub;
uint8_t       _PublishRetries=0;

//FORWARD DECLARATIONS
void PubSubCallback(char* pTopic, uint8_t* pData, unsigned int dalaLength);

void setup()
{
// Serial port for debugging purposes
	Serial.begin(115200);

	log_d("Program Starting, Setup...");

	pinMode(PIN_BAT_VOLTAGE, INPUT); //not really needed, all pins start as input
	adcAttachPin(PIN_BAT_VOLTAGE);
	analogReadResolution(10);
	analogSetPinAttenuation(PIN_BAT_VOLTAGE, ADC_11db); //max input value ~2600mv

	// Set all relays to off when the program starts - if set to Normally Open (NO), the relay is off when you set the relay to HIGH
	pinMode(PIN_RELAY, OUTPUT);
	if(RELAY_NO) {
		digitalWrite(PIN_RELAY, HIGH);
	}
	else {
		digitalWrite(PIN_RELAY, LOW);
	}

	//Configure Wi-Fi
	log_d("Connecting to WiFi (%s)...", WIFI_SSID);
	WiFi.begin(WIFI_SSID, WIFI_PASS);
	delay(2000);
	WiFi.setAutoReconnect(true);
	WiFi.persistent(true);
	if(WiFi.isConnected()) {
		log_d("Connected!!");
	}
	else {
		log_d("Not Connected to WiFi :(");
	}

	log_d("Setup Finished!");
}

uint16_t ReadAvgValue(uint8_t pin)
{
	uint8_t numreads = 5, count = 0;
	uint32_t value = 0;

	do {
		value += analogRead(pin);
		delay(20);
		count++;
	} while(count < numreads);

	return value / numreads;
}

void loop()
{
	auto value = ReadAvgValue(PIN_BAT_VOLTAGE);
	bool rdy=true;
	log_d("Pin value=%d", (int)value);

//max input voltage=2.426v--> 0,735% de 3.3v --> 3.3/2.426=1,36
//max battery voltage=4.25v
	auto volt = (1.36f * (float)value / 1023.0f) * MAX_VOLTAGE;
	auto percent = (volt - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE);
	if(percent<0) {
		percent=0;
	}
	log_d("Voltage=%3.2fv  Battery=%3.2f%%", (float)volt, (float)percent);

	if(!WiFi.isConnected()) {
		log_d("Reconnecting to Wifi...");
		WiFi.reconnect();
	}
	if(WiFi.status() != WL_CONNECTED) {
		log_d("Couldn't get a wifi connection!");
		rdy = false;
	}
	else {
		if(!_ThePubSub.connected()) {
			log_d("Connecting to MQTT broker [%s:%d]", MQTT_BROKER, MQTT_PORT);
			_ThePubSub.setClient(_TheWifi);
			_ThePubSub.setServer(MQTT_BROKER, MQTT_PORT);
			_ThePubSub.setCallback(PubSubCallback);
			if(!_ThePubSub.connect("PChanSmartPlug")) {
				log_d("ERROR!! PubSubClient was not able to connect to [%s:%d]", MQTT_BROKER, MQTT_PORT);
				rdy = false;
			}
			else { //Subscribe to the switch topic
				log_d("PubSubClient connected to [%s:%d]!!", MQTT_BROKER, MQTT_PORT);
				if(!_ThePubSub.subscribe(TOPIC_SWITCH)) {
					log_d("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_SWITCH);
				}
				//retrieve last value
				//_ThePubSub.
			}
		}
	}
	if(rdy) {
		_ThePubSub.publish(TOPIC_VOLTAGE, String((uint8_t)(percent*100)).c_str(), true); //retain last value!
	}

	if(_ThePubSub.connected()) {
		_ThePubSub.loop(); //allow the pubsubclient to process incoming messages
		_PublishRetries++;
		log_d("Connected loop [%d]", _PublishRetries);
		if(_PublishRetries>4) {
			_PublishRetries=0;
			//aaaand go to sleep
			log_d("Going to bed...");
			delay(5000);

			gpio_hold_en(GPIO_NUM_26);
			gpio_deep_sleep_hold_en();

			//Set timer to 5 seconds
			esp_sleep_enable_timer_wakeup(SLEEP_TIME_SECS * uS_TO_S_FACTOR);
			//Go to sleep now
			//esp_light_sleep_start();
			esp_deep_sleep_start();
		}
		//esp_deep_sleep_start();
	}
	else if(_PublishRetries < MAX_RETRIES) {
		delay(DELAY_LOOP_MS);
		_PublishRetries++;
	}
	else {
		log_d("Going to bed...");
	}

	delay(1000);
}

void PubSubCallback(char* pTopic, uint8_t* pData, unsigned int dataLenght)
{
	std::string theTopic(pTopic);
	std::string theMsg;

	for(uint16_t i = 0; i < dataLenght; i++) {
		theMsg.push_back((char)pData[i]);
	}
	log_d("Received message from [%s]: [%s]", theTopic.c_str(), theMsg.c_str());

	if(theTopic.find(TOPIC_SWITCH) != std::string::npos) {
		gpio_hold_dis(GPIO_NUM_26);
		if(theMsg == "ON") {
			log_d("Turning on the Relay!");
			if(RELAY_NO) {
				digitalWrite(PIN_RELAY, LOW);
			}
			else {
				digitalWrite(PIN_RELAY, HIGH);
			}
		}
		else {
			log_d("Turning off the Relay");
			if(RELAY_NO) {
				digitalWrite(PIN_RELAY, HIGH);
			}
			else {
				digitalWrite(PIN_RELAY, LOW);
			}
		}
	}
}
