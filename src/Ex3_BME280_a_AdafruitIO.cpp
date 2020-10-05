
#include <Arduino.h>
#include "AdafruitIO_WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"

#define ADAIO_USER "kaketa"
#define ADAIO_KEY "aio_CPly808RzG7Xu4rhtcsT3E0hNUEd"
#define WIFI_SSID "Picadero"
#define WIFI_PASS "Vegethalia"

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_LED_MIO 32
#define PIN_LED_PWM 25
#define PIN_MOTOR_PWM 17
#define I2C_ADDRESS_BME280 0x76

#define PWMCHANNEL 0
#define PWMRESOLUTION 8

#define FEED_TURN_ON_PWN "TurnOnLed"
#define FEED_INTENSITY_PWN "LedIntensity"

#define SEALEVELPRESSURE_HPA (1013.25)

//-----FORWARD DECLARATIONS
void uploadValues();
void onMessageOnOff(AdafruitIO_Data *data);
void FirstTimeProcess();
//-----FORWARD DECLARATIONS

AdafruitIO_WiFi iowifi(ADAIO_USER, ADAIO_KEY, WIFI_SSID, WIFI_PASS);

// set up the 'temp' feed
AdafruitIO_Feed *adafruit_temp = iowifi.feed("Temperatura");
AdafruitIO_Feed *adafruit_hum = iowifi.feed("Humitat");
AdafruitIO_Feed *adafruit_press = iowifi.feed("Pressio");
//AdafruitIO_Feed *adafruit_alt = iowifi.feed("Altitut");
AdafruitIO_Feed *adafruitread_blueledon = iowifi.feed(FEED_TURN_ON_PWN);
AdafruitIO_Feed *adafruitread_blueledpower = iowifi.feed(FEED_INTENSITY_PWN);

//Our own TwoWire instance so we can configure wich pins use as I2C
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long _delayTimeUpt=30000;
unsigned long _delayTimeLoop=500;
unsigned long _lastProcessMillis=0;

//state vars
bool _BlueLedON=false;
byte _BlueLedIntensity=128; //half intensity at 8 bits

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_LED_MIO, OUTPUT);
  pinMode(PIN_MOTOR_PWM, OUTPUT);

		//Configure PWM pin
	ledcSetup(PWMCHANNEL, 20000, PWMRESOLUTION); //channel0, freq=¿1000?, resolution=8bits
	ledcAttachPin(PIN_LED_PWM, PWMCHANNEL);
	//ledcAttachPin(PIN_MOTOR_PWM, PWMCHANNEL);

  // wait for serial monitor to open
  while(!Serial);

	//Detect BME
	Serial.println("Detecting BME280...");
	I2CBME.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  bool status = bme.begin(I2C_ADDRESS_BME280, &I2CBME);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
	Serial.println("BME280 sensor found!!");
  Serial.println();

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  iowifi.connect();
  // wait for a connection
  while(iowifi.status() < AIO_CONNECTED) {
    Serial.print("("); Serial.print(iowifi.status()); Serial.print(")"); 
    delay(500);
  }
  // we are connected
  Serial.println();
	Serial.print("iowifi Status="); Serial.println(iowifi.statusText());

  // set up a message handler for the 'sharedFeed' feed.
  // the handleMessage function (defined below)
  // will be called whenever a message is
  // received from adafruit io.
	if(adafruitread_blueledon) {
		adafruitread_blueledon->onMessage(onMessageOnOff);
	}
	if(adafruitread_blueledpower) {
		adafruitread_blueledpower->onMessage(onMessageOnOff);
	}
}

void loop() 
{
  // iowifi.run(); is required for all sketches.
  // it should always be present at the top of your loop function. 
  // it keeps the client connected to io.adafruit.com, and processes any incoming data.
  iowifi.run();

	auto now=millis();

	digitalWrite(PIN_MOTOR_PWM, HIGH);

	if((now-_lastProcessMillis)>=_delayTimeUpt) {
		_lastProcessMillis=now;

		digitalWrite(PIN_LED_MIO, HIGH);	// turn on the LED
		delay(200);	
		digitalWrite(PIN_LED_MIO, LOW);	// turn off the LED

	//	printValues();
		uploadValues();
	}

  delay(_delayTimeLoop);	

  // if(!adafruit_power->save(percent)) {
  //   Serial.println("Error updating AdaFruit Io Value :(");
  // }
}


void onMessageOnOff(AdafruitIO_Data *data)
{
	if(!data) {
		Serial.println("NULL DATA!!");
		return;
	}
	char buff[1000];

	snprintf(buff, sizeof(buff), "Message received from feed [%s]:[%s]", data->feedName(), data->value()); Serial.println(buff);

	if(strcmp(data->feedName(), FEED_TURN_ON_PWN)==0) {
		if(strcmp(data->toChar(), "ON")==0) {
			Serial.print("Encenent el blue led, intensitat="); Serial.println(_BlueLedIntensity);
			_BlueLedON=true;
			ledcWrite(PWMCHANNEL, _BlueLedIntensity);
		}
		else {
			Serial.println("Apagant el blue led...");
			_BlueLedON=false;
			ledcWrite(PWMCHANNEL, 0);
		}
	}
	else if(strcmp(data->feedName(), FEED_INTENSITY_PWN)==0) {
		snprintf(buff, sizeof(buff), "Rebuda intensitat de led=%d", data->toInt());
		Serial.println(buff);
		if(data->toInt()<0) {
			_BlueLedIntensity=0;
		}
		else if(data->toInt()>255) {
			_BlueLedIntensity=255;
		}
		else {
			_BlueLedIntensity=data->toInt();
		}
		if(_BlueLedON) {
			ledcWrite(PWMCHANNEL, _BlueLedIntensity);
		}
	}
}

void uploadValues()
{
	char buffer[100];
	float temp=bme.readTemperature();
	float hum=bme.readHumidity();
	float press=bme.readPressure()/100.0;
//	float alt=bme.readAltitude(SEALEVELPRESSURE_HPA);

	if(!isnan(temp) && temp>(-20) && temp<100 && !adafruit_temp->save(temp)) {
		Serial.println("Error updating temperature :(");
	}
	sprintf(buffer, "Temperature = %2.1fºC", temp);
	Serial.println(buffer);

	if(!isnan(hum) && hum>=0 && hum<=100 && !adafruit_hum->save(hum)) {
		Serial.println("Error updating humnidity :(");
	}
	sprintf(buffer, "Humidity = %2.1f%%", hum);
	Serial.println(buffer);

	if(!isnan(press) && press>900 && press<1100 && !adafruit_press->save(press)) {
		Serial.println("Error updating pressure :(");
	}
	sprintf(buffer, "Pressure = %3.1fhpa", press);
	Serial.println(buffer);

	// if(!adafruit_alt->save(alt)) {
	// 	Serial.println("Error updating altitude :(");
	// }
	// sprintf(buffer, "Altitude = %3.1fm", alt);
	//Serial.println(buffer);

	Serial.println();
}

void FirstTimeProcess()
{
	if(adafruitread_blueledon) {
		onMessageOnOff(adafruitread_blueledon->lastValue()); 	//get the last values published
	}
	if(adafruitread_blueledpower) {
		onMessageOnOff(adafruitread_blueledpower->lastValue()); 	//get the last values published
	}	
	// if(!adafruit_temp->exists()) {
	//   Serial.println("Temperature feed not found!!");
	// }
	// if(!adafruit_hum->exists()) {
	//   Serial.println("Humnidity feed not found!!");
	// }
	// if(!adafruit_press->exists()) {
	//   Serial.println("Pressure feed not found!!");
	// }
	// if(!adafruit_alt->exists()) {
	//   Serial.println("Altitude feed not found!!");
	// }

}
