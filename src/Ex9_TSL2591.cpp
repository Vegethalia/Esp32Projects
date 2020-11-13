/* TSL2591 Digital Light Sensor, example with (simple) interrupt support  */
/* Dynamic Range: 600M:1 */
/* Maximum Lux: 88K */

/*  This example shows how the interrupt system on the TLS2591
 *  can be used to detect a meaningful change in light levels.
 *  
 *  Two thresholds can be set: 
 *  
 *  Lower Threshold - Any light sample on CHAN0 below this value
 *                    will trigger an interrupt
 *  Upper Threshold - Any light sample on CHAN0 above this value
 *                    will trigger an interrupt
 *                    
 *  If CHAN0 (full light) crosses below the low threshold specified,
 *  or above the higher threshold, an interrupt is asserted on the interrupt
 *  pin. The use of the HW pin is optional, though, since the change can
 *  also be detected in software by looking at the status byte via
 *  tsl.getStatus().
 *  
 *  An optional third parameter can be used in the .registerInterrupt
 *  function to indicate the number of samples that must stay outside
 *  the threshold window before the interrupt fires, providing some basic
 *  debouncing of light level data.
 *  
 *  For example, the following code will fire an interrupt on any and every
 *  sample outside the window threshold (meaning a sample below 100 or above
 *  1500 on CHAN0 or FULL light):
 *  
 *    tsl.registerInterrupt(100, 1500, TSL2591_PERSIST_ANY);
 *  
 *  This code would require five consecutive changes before the interrupt
 *  fires though (see tls2591Persist_t in Adafruit_TLS2591.h for possible
 *  values):
 *  
 *    tsl.registerInterrupt(100, 1500, TSL2591_PERSIST_5);
 */
/*
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

// Example for demonstrating the TSL2591 library - public domain!

// connect SCL to I2C Clock
// connect SDA to I2C Data
// connect Vin to 3.3-5V DC
// connect GROUND to common ground

// Interrupt thresholds and persistance
#define TLS2591_INT_THRESHOLD_LOWER  (100)
#define TLS2591_INT_THRESHOLD_UPPER  (1500)
#define TLS2591_INT_PERSIST        (TSL2591_PERSIST_ANY) // Fire on any valid change
//#define TLS2591_INT_PERSIST          (TSL2591_PERSIST_60)  // Require at least 60 samples to fire

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

//    Configures the gain and integration time for the TSL2591
void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  // tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
   tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  // Display the gain and integration time for reference sake
  log_d("------------------------------------");
  tsl2591Gain_t gain = tsl.getGain();
  switch (gain)  {
    case TSL2591_GAIN_LOW:
      log_d("Gain: 1x (Low)");
      break;
    case TSL2591_GAIN_MED:
      log_d("Gain: 25x (Medium)");
      break;
    case TSL2591_GAIN_HIGH:
      log_d("Gain: 428x (High)");
      break;
    case TSL2591_GAIN_MAX:
      log_d("Gain: 9876x (Max)");
      break;
  }
  log_d("Timing:       %dms", (tsl.getTiming() + 1) * 100);
  log_d("------------------------------------");

  // Setup the SW interrupt to trigger between 100 and 1500 lux
  tsl.clearInterrupt();
  tsl.registerInterrupt(TLS2591_INT_THRESHOLD_LOWER,
                        TLS2591_INT_THRESHOLD_UPPER,
                        TLS2591_INT_PERSIST);
}


void setup(void)
{
  Serial.begin(115200);
  // wait for serial monitor to open
  while(!Serial) { delay(1); }

  if(tsl.begin())  {
    log_d("Found a TSL2591 sensor");
  }
  else {
    log_d("No sensor found ... check your wiring?");
  }

  // Configure the sensor (including the interrupt threshold) 
  configureSensor();
}

//Show how to read IR and Full Spectrum at once and convert to lux
void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
  Serial.print("IR: "); Serial.print(ir);  Serial.print("  ");
  Serial.print("Full: "); Serial.print(full); Serial.print("  ");
  Serial.print("Visible: "); Serial.print(full - ir); Serial.print("  ");
  Serial.print("Lux: "); Serial.println(tsl.calculateLux(full, ir));
}


void getStatus(void)
{
  uint8_t x = tsl.getStatus();
  // bit 4: ALS Interrupt occured
  // bit 5: No-persist Interrupt occurence
  if (x & 0x10) {
    Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
    Serial.println("ALS Interrupt occured");
  }
  if (x & 0x20) {
    Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
    Serial.println("No-persist Interrupt occured");
  }

  // Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
  Serial.print("Status: ");
  Serial.println(x, BIN);
  tsl.clearInterrupt();
}

void loop(void)
{
  advancedRead();
  getStatus();
  delay(2000);
}
*/