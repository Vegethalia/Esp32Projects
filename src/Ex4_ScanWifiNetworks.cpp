/*
#include <Arduino.h>
#include "WiFi.h"
#include "mykeys.h"

//EXAMPLE BASED ON https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiScan/WiFiScan.ino

void setup()
{
	Serial.begin(115200);

	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.mode(WIFI_STA);
	WiFi.disconnect();

	// wait for serial monitor to open
	while(!Serial);

	Serial.println("Setup done");
}

void loop()
{
	char buff[1000];

	if(WiFi.isConnected()) {
		WiFi.mode(WIFI_STA);
		WiFi.disconnect();
	}

	Serial.println("scan start");

	// WiFi.scanNetworks will return the number of networks found
	int n = WiFi.scanNetworks();
	Serial.println("scan done");
	if (n == 0) {
		Serial.println("no networks found");
	} else {
		Serial.print(n);
		Serial.println(" networks found");
		for (int i = 0; i < n; ++i) { //Print SSID and RSSI for each network found
			snprintf(buff, sizeof(buff), "%2d: %s (%ddb) Channel=%d %s", 
				i, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i), (WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
			Serial.println(buff);

			delay(10);
		}
	}
	Serial.println("");
	Serial.println("Connecting to WIFI...");

	WiFi.mode(WIFI_MODE_AP);
	WiFi.disconnect();

	WiFi.begin(WIFI_SSID, WIFI_PASS);
	while(WiFi.status() != WL_CONNECTED) {
  	delay(500);
  	Serial.print(".");
	}
 	Serial.println("");
	snprintf(buff, sizeof(buff), "Connected!! IP=[%s]  Gateway=[%s] DNS=[%s]", 
		WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str(), WiFi.dnsIP().toString().c_str());
	Serial.println(buff);

	// Wait a bit before scanning again
	Serial.println("");
	Serial.println("Press 'R' to scan again.");
	do {
		if((Serial.available()>0)) {
			auto inc=Serial.read();

			Serial.print("I received: ");
			Serial.println(inc, DEC);

			if(inc=='R') {
				break;
			}
		}

		delay(5000);
	} while(true);
}
*/