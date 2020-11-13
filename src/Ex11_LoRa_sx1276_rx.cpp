/*
#include <Arduino.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#define PIN_LED 32

//define the pins used by the transceiver module
#define PIN_SS        5
#define PIN_RST       16
#define PIN_DIO0      17
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define I2C_BUS_SPEED 100000 //100000

//SetUp SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA); //R0 = no rotate

int _counter = 0;

void PrintValuesOnScreen(bool newMsg, String theMsg, int RSSI=0, float SNR=0.0)
{
	char buffer[128];
	int8_t fh = 10, h;

	u8g2.firstPage();
	do {
		//		u8g2.clearBuffer();
		u8g2.setFont(u8g2_font_sirclivethebold_tr); //7 pixels
		snprintf(buffer, sizeof(buffer), "LoRa Test");
		u8g2.setCursor(15, fh);
		u8g2.print(buffer);

		if(newMsg) {
			u8g2.setFont(u8g2_font_profont11_mf); //7 pixels monospace
			h = fh * 2.4;
			u8g2.setCursor(0, h);
			snprintf(buffer, sizeof(buffer), "Message number: %d", _counter);
			u8g2.print(buffer);
			u8g2.setCursor(0, 64);
			snprintf(buffer, sizeof(buffer), "RSSI:%d  SNR:%2.1f", RSSI, SNR);
			u8g2.print(buffer);

			u8g2.setFont(u8g2_font_6x10_mf);
			h += fh * 1.25;
			u8g2.setCursor(0, h);
			snprintf(buffer, sizeof(buffer), "%s", theMsg.substring(0, 7).c_str());
			u8g2.print(buffer);
			h += fh * 1.25;
			u8g2.setCursor(0, h);
			snprintf(buffer, sizeof(buffer), "%s", theMsg.substring(7).c_str());
			u8g2.print(buffer);
//			h += fh * 1.25;
		}
		else {
			u8g2.setFont(u8g2_font_profont11_mf); //7 pixels monospace
			h = fh * 4.4;
			u8g2.setCursor(10, h);
			snprintf(buffer, sizeof(buffer), "WAITING");
			u8g2.print(buffer);
		}

		// u8g2.setFont(u8g2_font_profont10_mf); //6 pixels monospace
		// snprintf(buffer, sizeof(buffer), "%c %2ds %c     [%3dms]", c, millisPending / 1000, c, _lastAvgUpdateTime);
		// u8g2.drawStr(15, 64, buffer);
	} while (u8g2.nextPage());
	//u8g2.sendBuffer();
}

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while (!Serial)
	{
		delay(100);
	}

	pinMode(PIN_LED, OUTPUT);

	//setup LoRa transceiver module
	LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
	Serial.println("LoRa - Initializing frequency...");
	while (!LoRa.begin(866E6))
	{
		Serial.println(".");
		delay(500);
	}
	// Change sync word (0xF3) to match the receiver
	// The sync word assures you don't get LoRa messages from other LoRa transceivers
	// ranges from 0-0xFF
	LoRa.setSyncWord(0x11);			 //17 :)
	LoRa.setSpreadingFactor(12); //lent però segur... :D
	LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
	LoRa.setCodingRate4(6);
	Serial.println("LoRa Initializing OK!");

	log_d("Begin Display...");
	u8g2.setBusClock(I2C_BUS_SPEED);
	//u8g2.beginSimple();			// does not clear the display and does not wake up the display  user is responsible for calling clearDisplay() and setPowerSave(0)
													//  u8g2.begin();
	u8g2.begin();
	u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print()
	u8g2.setContrast(100); //set default contrast
	PrintValuesOnScreen(false, "");
}

void loop()
{
	// try to parse packet
	int packetSize = LoRa.parsePacket();
	if (packetSize)	{
		String LoRaData;
		
		// read packet
		while (LoRa.available()) {
			LoRaData += LoRa.readString();
		}

		char buff[255];
		snprintf(buff, sizeof(buff), "Received message [%s]. RSSI=%d SNR=%2.1f", LoRaData.c_str(), LoRa.packetRssi(), LoRa.packetSnr());
		Serial.println(buff);

		_counter++;
		PrintValuesOnScreen(true, LoRaData, LoRa.packetRssi(), LoRa.packetSnr());
	}
}
*/