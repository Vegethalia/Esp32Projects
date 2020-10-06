/*
//EXAMPLE2 --> Sent to AdaFruitIO the values of a Potenciometer

#include <Arduino.h>
#include "AdafruitIO_WiFi.h"
#include "Adafruit_MQTT.h"
#include "mykeys.h"

#define PIN_LED_MIO 32
#define PIN_POT 34

#define MAX_TIME_LED_ON 2000 //in millisecs

AdafruitIO_WiFi iowifi(ADAIO_USER, ADAIO_KEY, WIFI_SSID, WIFI_PASS);

// set up the 'power' feed
AdafruitIO_Feed *adafruit_power = iowifi.feed("Potencia");

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_LED_MIO, OUTPUT);
  pinMode(PIN_POT, INPUT); //not really needed, all pins start as input


  // wait for serial monitor to open
  while(!Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  iowifi.connect();

  // wait for a connection
  while(iowifi.status() < AIO_CONNECTED) {
    Serial.print("("); Serial.print(iowifi.status()); Serial.print(")"); 
    delay(500);
  }
  // we are connected
  Serial.println();
  Serial.println(iowifi.statusText());
}

void loop() {
  int16_t pot=0;
  float percent=0.5f;

  // iowifi.run(); is required for all sketches.
  // it should always be present at the top of your loop function. 
  // it keeps the client connected to io.adafruit.com, and processes any incoming data.
  iowifi.run();

  digitalWrite (PIN_LED_MIO, HIGH);	// turn on the LED

  pot=analogRead(PIN_POT);
  Serial.print("Value=");
  Serial.println(pot);

  //scale pot to 0.0...1.0
  percent=pot/(float)4095;

  delay(MAX_TIME_LED_ON*percent);	// wait for time proportional to potentiometer

  digitalWrite (PIN_LED_MIO, LOW);	// turn off the LED
  delay(MAX_TIME_LED_ON*percent);	// wait for time proportional to potentiometer

  if(!adafruit_power->save(percent)) {
    Serial.println("Error updating AdaFruit Io Value :(");
  }
}
*/