/*
#include <Arduino.h>
#include <LoRa.h>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#define PIN_LED 32

//define the pins used by the transceiver module
#define PIN_SS   5
#define PIN_RST  16
#define PIN_DIO0 17

int _counter=0;

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while (!Serial) {delay(100);}

	pinMode(PIN_LED, OUTPUT);

	//setup LoRa transceiver module
	LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
	Serial.println("LoRa - Initializing frequency...");
	while (!LoRa.begin(866E6))	{
		Serial.println(".");
		delay(500);
	}
	// Change sync word (0xF3) to match the receiver
	// The sync word assures you don't get LoRa messages from other LoRa transceivers
	// ranges from 0-0xFF
	LoRa.setSyncWord(0x11); //17 :)
	LoRa.setSpreadingFactor(12); //lent però segur... :D
	LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
	LoRa.setCodingRate4(6);
	Serial.println("LoRa Initializing OK!");
}

void loop()
{
	Serial.print("Sending packet: ");
	Serial.println(_counter);
	digitalWrite(PIN_LED, HIGH);

	//Send LoRa packet to receiver
	char buff[255];
	snprintf(buff, sizeof(buff), "[%lus] Hello server! (%04d)", millis()/1000, _counter);

	LoRa.beginPacket();
	LoRa.print(buff);
	LoRa.endPacket();

	digitalWrite(PIN_LED, LOW);

	_counter++;

	delay(10000);
}
*/