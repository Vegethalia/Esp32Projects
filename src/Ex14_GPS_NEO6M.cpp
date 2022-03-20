/*
#include <Arduino.h>
#include <Wire.h>
//#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <vector>
#include <memory>
#include <ctime>
#include <string>
#include <NMEAGPS.h>
#include <SoftwareSerial.h>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.
#include "modules/ScreenDebugger.h"
#include "types.h"
#include "modules/Utils.h"


#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// ADAFRUIT Feeds / Related stuff
#define ADAFRUIT_ADDR      "52.7.124.212" //"io.adafruit.com"
#define ADAFRUIT_PORT      1883

#define FEED_LOCATION      "/feeds/location/csv" //sensor_value,latitude,longitude,elevation

//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C _u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);

#define PIN_LED 32
#define GSMPIN_RX 19
#define GSMPIN_TX 18
#define CHECK_GPRS_EVERY_MS 20000

#define TINY_GSM_MODEM_SIM800 // definição do modem usado (SIM800L)
#include <TinyGsmClient.h>		// biblioteca com comandos GSM

//HardwareSerial SerialGSM(2);
SoftwareSerial  _SerialGSM(GSMPIN_RX, GSMPIN_TX); // objeto de comunicação serial do SIM800L
TinyGsm         _modemGSM(_SerialGSM);
TinyGsmClient   _clientGSM(_modemGSM);

HardwareSerial  _SerialGPS(2);
TwoWire         _I2Cscreen = TwoWire(0);
//WiFiClient      _TheWifi;
PubSubClient    _ThePubSub;

//ScreenDebugger  _TheDebug(&_u8g2, 5, 1);
ScreenInfo      _TheScreenInfo;
bool            _ScreenActive = true;

NMEAGPS         _TheNeoGps; // This parses the GPS characters
gps_fix         _TheFix;


//TIMING VARS
unsigned long _delayTimeUpt = 30000;
unsigned long _lastProcessMillis = 0;
unsigned long _LastFixed = 0;

// // objeto da bibliteca com as funções GSM
// TinyGsm modemGSM(SerialGSM);
// velocidade da serial tanto do SIM800L quanto do monitor serial
//const int BAUD_RATE = 28800;//9600;
// pinos aonde os reles serão ligados e RX / TX aonde o SIM800L será ligado
//const int RX_PIN = 4, TX_PIN = 2;

int _timeout = 0;

//Envia comando AT e aguarda até que uma resposta seja obtida
// String sendAT(String command)
// {
// 	String response = "";
// 	_SerialGSM.println(command);
// 	// aguardamos até que haja resposta do SIM800L
// 	while(!_SerialGSM.available());

// 	response = _SerialGSM.readString();

// 	return response;
// }

void setupGSM()
{
	log_d("Setup GSM...");

	// inicia serial SIM800L
	//SerialGSM.begin(BAUD_RATE);//, SERIAL_8N1, RX_PIN, TX_PIN, false);

			// Set GSM module baud rate
	TinyGsmAutoBaud(SerialGSM, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
	//delay(3000);

	log_d("Modem NAME: %s", modemGSM.getModemName().c_str());
	log_d("Modem INFO: %s", modemGSM.getModemInfo().c_str());
	log_d("Modem IMEI: %s", modemGSM.getIMEI().c_str());

	// if(modemGSM.simUnlock("1234")) {
	// 	Serial.println("SIM UNLOCKED");
	// }
	// else {
	// 	Serial.println("SIM LOCKED");
	// }

	log_d("SignalQ: %d", modemGSM.getSignalQuality());
	log_d("SIM status: %s", modemGSM.getSimStatus() ? "READY" : "LOCKED");
	log_d("COPS? --> [%s]", sendAT("AT+COPS=?").c_str());
	log_d("CREG? --> [%s]", sendAT("AT+CREG?").c_str());
	log_d("CBAND? --> [%s]", sendAT("AT+CBAND=?").c_str());

	//aguarda network
	if(!modemGSM.waitForNetwork(30000)) {
		log_d("Failed to connect to network");
		if(!modemGSM.restart()) {
			log_d("Restarting GSM\nModem failed");
		}
		else {
			log_d("GSM Modem restarted");
		}
		ESP.restart();
		return;
	}
	log_d("Modem registered to network OK!!");

	//conecta na rede (tecnologia GPRS)
	if(!modemGSM.gprsConnect("internetmas")) {
		log_d("GPRS Connection Failed :(");
		delay(10000);
		ESP.restart();
		return;
	}
	log_d("GPRS Connected OK!!");
}

// verifica se o SIM800L se desconectou, se sim tenta reconectar
bool verifyGPRSConnection()
{
	bool result = false;

	if(modemGSM.isGprsConnected()) {
		result = true;
		log_d("GPRS: Connected!");
	}
	else {
		log_d("GPRS: Disconnected. Reconnecting...");

		if(!modemGSM.waitForNetwork()) {
			log_d("Network Failed");
		}
		else {
			if(!modemGSM.gprsConnect(APN)) {
				log_d("GPRS Failed");
			}
			else {
				result = true;
				log_d("GPRS Con. OK");
			}
		}
	}
	return result;
}

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while(!Serial);

	log_d("Setup GPS...");
	_SerialGPS.begin(9600);

	pinMode(PIN_LED, OUTPUT);

	_I2Cscreen.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000); //0=default 100000 100khz

	log_d("Begin Display...");
	_u8g2.setBusClock(100000);
	_u8g2.begin();
	_u8g2.setFont(u8g2_font_5x8_mf);
	//_TheDebug.SetFont(ScreenDebugger::SIZE1);

	// inicia e configura o SIM800L
	setupGSM();

	//Initialize Wifi
	//_TheDebug.NewLine("Initializing WiFi...");
	log_d("Initializing WiFi...");

	WiFi.setAutoReconnect(true);
	//WiFi.config(_LocalIP, _GWIP, _MaskIP, _DNS1);
	wl_status_t statuswf = WiFi.begin(WIFI_SSID, WIFI_PASS);
	WiFi.waitForConnectResult();
	if(statuswf != WL_CONNECTED) {
		_TheScreenInfo.wifiState = "Connecting...";
		//_TheDebug.NewLine("NOT CONNECTED :(");
		log_d("NOT CONNECTED :(");
	}
	else {
//		WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(), IPAddress(8, 8, 8, 8));
		_TheScreenInfo.wifiState = WiFi.localIP().toString().c_str();
		// _TheDebug.NewLine("WIFI CONNECTED!!");
		// _TheDebug.NewLine(WiFi.localIP().toString().c_str());
		log_d("CONNECTED! IP=[%s]", WiFi.localIP().toString().c_str());
	}

//	_TheDebug.NewLine("Setup Complete!");
	log_d("Setup Complete!");
}

void DrawScreen()
{
	if(!_ScreenActive) {
		return;
	}

	uint16_t charH = _u8g2.getMaxCharHeight() == 0 ? 10 : _u8g2.getMaxCharHeight();
	uint16_t maxHeight = _u8g2.getHeight();
	uint16_t j = charH;

	std::tm* ptm = std::localtime(&_TheScreenInfo.gps_time);
	char tbuffer[32];
	std::strftime(tbuffer, sizeof(tbuffer), "%d/%m/%Y %H:%M:%S", ptm);

	_u8g2.firstPage();
	do {
		j = charH;
		_u8g2.setCursor(0, j); j += charH * 2;
		if(_TheScreenInfo.gpsFix) {
			_u8g2.print("GPS: F I X E D");
		}
		else {
			_u8g2.print("GPS: Locating...");
		}
		_u8g2.setCursor(0, j); j += charH;
		_u8g2.print(Utils::string_format("Lat/Lon: (%3.3f, %3.3f)", _TheScreenInfo.lat_deg, _TheScreenInfo.lon_deg).c_str());
		_u8g2.setCursor(0, j); j += charH;
		_u8g2.print(Utils::string_format("Alt: %3.1fm Spd: %3.1fkph", _TheScreenInfo.alt_m, _TheScreenInfo.speed_kph).c_str());
		_u8g2.setCursor(0, j); j += charH;
		_u8g2.print(Utils::string_format("Wifi: %s", _TheScreenInfo.wifiState.c_str()).c_str());
		_u8g2.setCursor(0, j); j += charH * 2;
		_u8g2.print(Utils::string_format("Mqtt: %s", _TheScreenInfo.mqttState.c_str()).c_str());
		_u8g2.setCursor(0, j); j += charH;
		_u8g2.print(Utils::string_format("Time: %s", tbuffer).c_str());

	} while(_u8g2.nextPage());
}

void loop()
{
	auto now = millis();
	if(_TheScreenInfo.gpsFix && (now - _LastFixed) > 60000) {
		_TheScreenInfo.gpsFix = false;
	}

	while(_TheNeoGps.available(_SerialGPS)) {
		_TheFix = _TheNeoGps.read();
		if(_TheFix.valid.location) {
			_LastFixed = now;
			_TheScreenInfo.gpsFix = true;
			_TheScreenInfo.lat_deg = _TheFix.latitude();
			_TheScreenInfo.lon_deg = _TheFix.longitude();
			//snprintf(buff, sizeof(buff), " Fix! (%3.2f,%3.2f) ", _TheFix.latitude(), _TheFix.longitude());
			//_TheDebug.NewLine(buff);
			// snprintf(buff, sizeof(buff), "Alt=%d Sp=%3.2f", _TheFix.altitude_cm(), _TheFix.speed_kph());
			// _TheDebug.NewLine(buff);
		}
		if(_TheFix.valid.speed) {
			_TheScreenInfo.speed_kph = _TheFix.speed_kph();
		}
		if(_TheFix.valid.altitude) {
			_TheScreenInfo.alt_m = _TheFix.altitude();
		}
		if(_TheFix.valid.date && _TheFix.valid.time) {
			_TheScreenInfo.gps_time = (NeoGPS::clock_t)_TheFix.dateTime;
		}
		// else {
		// 	_TheScreenInfo.gpsFix=false;
		// 	//_TheDebug.NewLine("Not Fixed :(");
		// }
	}


	if((now - _lastProcessMillis) >= _delayTimeUpt) {
		bool rdy = true;
		_lastProcessMillis = now;

		log_d("Time 2 update!!");

		if(!WiFi.isConnected()) {
			//_TheDebug.NewLine("Reconnecting Wifi...");
			log_d("Reconnecing WIFI...");
			WiFi.reconnect();
		}
		if(WiFi.status() != WL_CONNECTED) {
			_TheScreenInfo.wifiConnected = false;
			_TheScreenInfo.wifiState = "Connecting...";
			// _TheDebug.NewLine("Not Connected :(");
			log_d("Wifi Not Connected :(");
			rdy = false;
		}
		else {
			_TheScreenInfo.wifiConnected = true;
			_TheScreenInfo.wifiState = WiFi.localIP().toString().c_str();

			log_d("Wifi Connected. IP=%s DNS=%s", WiFi.localIP().toString().c_str(), WiFi.dnsIP().toString().c_str());
			//WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(), IPAddress(8, 8, 8, 8));
			//_TheDebug.NewLine("WIFI CONNECTED!!");
			//_TheDebug.NewLine(WiFi.localIP().toString().c_str());

			if(!_ThePubSub.connected()) {
				log_d("PubSubClient Not Connected :(");

				_ThePubSub.setClient(_clientGSM);//_TheWifi);
				_ThePubSub.setServer(ADAFRUIT_ADDR, ADAFRUIT_PORT);
				//_ThePubSub.setCallback(PubSubCallback);
				if(!_ThePubSub.connect("PChanMQTT", ADAIO_USER, ADAIO_KEY)) {
					log_d("ERROR!! PubSubClient was not able to connect to AdafruitIO!!");
					//_TheDebug.NewLine("MQTT error :(");
					_TheScreenInfo.mqttConnected = false;
					_TheScreenInfo.mqttState = Utils::string_format("Error. State=%d", _ThePubSub.state());
					rdy = false;
				}
				else {
					_TheScreenInfo.mqttConnected = true;
					_TheScreenInfo.mqttState = "Connected";
					log_d("PubSubClient connected to AdafruitIO!!");
					//_TheDebug.NewLine("MQTT OK!!");
				}
			}
		}
		if(rdy) {
			if(_TheFix.valid.location) {
				std::string msg = Utils::string_format("%f, %f, %f, %d", _TheFix.speed_kph(), _TheFix.latitude(), _TheFix.longitude(), _TheFix.altitude_cm() / 100);
				log_d("Publishing [%s]", msg.c_str());
				if(_ThePubSub.publish((std::string(ADAIO_USER).append(FEED_LOCATION)).c_str(), msg.c_str())) {
					_TheScreenInfo.mqttState = Utils::string_format("(%3.2f,%3.2f)", _TheFix.latitude(), _TheFix.longitude());
				}
				else {
					_TheScreenInfo.mqttState = "Publish Error";
				}
			}
			else {
				log_d("Not valid location :(");
			}
		}
	}
	if(_ThePubSub.connected()) {
		_ThePubSub.loop(); //allow the pubsubclient to process incoming messages
	}
	DrawScreen();
}
*/
