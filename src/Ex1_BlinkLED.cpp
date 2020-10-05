/* #include <Arduino.h>

//EXAMPLE1: BLINK a LED!!!

#define LED_BUILTIN 2
#define LED_MIO 32

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_MIO, OUTPUT);
}

void loop() {
  Serial.println("Start loop");
  digitalWrite (LED_MIO, HIGH);	// turn on the LED
  delay(500);	// wait for half a second or 500 milliseconds
  digitalWrite (LED_MIO, LOW);	// turn off the LED
  delay(500);	// wait for half a second or 500 milliseconds
  Serial.println("Exiting loop");
}
*/
