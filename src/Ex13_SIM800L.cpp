#include <Arduino.h>
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.

#define TINY_GSM_MODEM_SIM800 // definição do modem usado (SIM800L)

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG Serial

// Range to attempt to autobaud
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 57600

#include <TinyGsmClient.h>		// biblioteca com comandos GSM


#define PIN_LED 32
//#define DUMP_AT_COMMANDS
#define CHECK_GPRS_EVERY_MS 20000

// objeto de comunicação serial do SIM800L
HardwareSerial SerialGSM(2);

// // objeto da bibliteca com as funções GSM
// TinyGsm modemGSM(SerialGSM);
// velocidade da serial tanto do SIM800L quanto do monitor serial
//const int BAUD_RATE = 28800;//9600;
// pinos aonde os reles serão ligados e RX / TX aonde o SIM800L será ligado
//const int RX_PIN = 4, TX_PIN = 2;

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialGSM, Serial);
TinyGsm modemGSM(debugger);
#else
TinyGsm modemGSM(SerialGSM);
#endif

int _timeout=0;

//Envia comando AT e aguarda até que uma resposta seja obtida
String sendAT(String command)
{
	String response = "";
	SerialGSM.println(command);
	// aguardamos até que haja resposta do SIM800L
	while (!SerialGSM.available());

	response = SerialGSM.readString();

	return response;
}

// inicializa GSM
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
	log_d("SIM status: %s", modemGSM.getSimStatus()?"READY":"LOCKED");
	log_d("COPS? --> [%s]", sendAT("AT+COPS=?").c_str());
	log_d("CREG? --> [%s]", sendAT("AT+CREG?").c_str());
	log_d("CBAND? --> [%s]", sendAT("AT+CBAND=?").c_str());

	//aguarda network
	if (!modemGSM.waitForNetwork(30000)) {
		log_d("Failed to connect to network");
		if (!modemGSM.restart()){
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
	if (!modemGSM.gprsConnect("internetmas"))	{
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
	bool result=false;

	if (modemGSM.isGprsConnected()) {
		result=true;
		log_d("GPRS: Connected!");
	}
	else {
		log_d("GPRS: Disconnected. Reconnecting...");

		if (!modemGSM.waitForNetwork())	{
			log_d("Network Failed");
		}
		else	{
			if (!modemGSM.gprsConnect(APN))	{
				log_d("GPRS Failed");
			}
			else	{
				result = true;
				log_d("GPRS Con. OK");
			}
		}
	}
	return result;
}

void GetGPS()
{
	float lat = 0;
	float lon = 0;
	float accuracy = 0;
	int year = 0;
	int month = 0;
	int day = 0;
	int hour = 0;
	int min = 0;
	int sec = 0;
	log_d("Requesting current GSM location");
	if (modemGSM.getGsmLocation(&lat, &lon, &accuracy, &year, &month, &day, &hour, &min, &sec))	{
		log_d("Latitude: %3.3f \tLongitude: %3.3f", lat, lon);
		log_d("Accuracy: %3.1f", accuracy);
		log_d("Date: %d/%d/%d %d:%d%d", year, month, day, hour, min, sec);
	}
	else {
		log_d("Couldn't get GSM location");
	}
	log_d("Retrieving GSM location again as a string: %s", modemGSM.getGsmLocation());
}

void setup()
{
	Serial.begin(115200);
	// wait for serial monitor to open
	while (!Serial);

	pinMode(PIN_LED, OUTPUT);

	// inicia e configura o SIM800L
	setupGSM();
}

void loop()
{
	String msg, number;

	if (SerialGSM.available()) {
		Serial.write(SerialGSM.read());
	}
	if (Serial.available())	{
		SerialGSM.write(Serial.read());
	}

	if ((millis() - _timeout) >= CHECK_GPRS_EVERY_MS)
	{
		_timeout=millis();
		if(verifyGPRSConnection()) {
			GetGPS();
		}
	}

	// se o SIM800L está conectado
//	if (modemGSM.isGprsConnected())	{
//	}
	// único delay no loop de 10ms (desconsiderando a função de reconexão, que possui delay para exibição do display)
	delay(10);
}

