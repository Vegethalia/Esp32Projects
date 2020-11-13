/*
#include <Arduino.h>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#define PIN_LED 32
#define PIN_BUTTON 35
#define PIN_PIR 14

#define TIME_WITHOUT_DETECTION 60 //in seconds

struct MovDetection {
	bool Interrupted;
	unsigned long LastDetectionTime;
};
MovDetection _DetectionState{false, 0};

void IRAM_ATTR MovementDetected()
{
	char buff[128];
	Serial.println("INTERRUPTED BY AM312!!");

	int pinValue = digitalRead(PIN_PIR);
	snprintf(buff, sizeof(buff), "[%d] Pin value=%d", (int)millis(), pinValue);
	Serial.println(buff);

	_DetectionState.Interrupted = true;
	_DetectionState.LastDetectionTime = millis();
}

void setup()
{
 	Serial.begin(115200);
  // wait for serial monitor to open
  while(!Serial);

	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_BUTTON, INPUT);
	pinMode(PIN_PIR, INPUT_PULLUP);

	attachInterrupt(PIN_PIR, MovementDetected, RISING);
}

void loop()
{
	if(_DetectionState.Interrupted) {
		digitalWrite(PIN_LED, HIGH);
		_DetectionState.Interrupted=false;
	}
	if(((millis()- _DetectionState.LastDetectionTime)/1000)>=TIME_WITHOUT_DETECTION) {
		digitalWrite(PIN_LED, LOW);
	}
}
*/