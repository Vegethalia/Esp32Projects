#include <Arduino.h>
//#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

U8G2_SH1106_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, PIN_I2C_SCL, PIN_I2C_SDA);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins) . Reset pin=-1 (screen does not have reset pin)
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup()
{
 	Serial.begin(115200);
  // wait for serial monitor to open
  while(!Serial);

  log_d("Begin Display...");
  u8g2.begin();

  //For SSD1306 driver use ADAFRUIT library!
	//  if(!display.begin(SSD1306_SWITCHCAPVCC, 0)) { // Address 0x3D for 128x64
  //    log_d("SSD1306 allocation failed");
  //  }
  //  else {
	//  delay(2000);
  //  display.clearDisplay();
  //  display.setTextSize(1);
  //  display.setTextColor(WHITE);
  //  display.setCursor(0, 10);
  //  display.println("Hello, world!");// Display static text
  //  display.display(); 
  // }
}

void loop()
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont10_mf);
    u8g2.drawStr(0,6,"Soc Peque 6px");
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0,24,"Hola Bola!");
    u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
    u8g2.drawStr(0,30,"Soc Peque 5px");
    u8g2.setFont(u8g2_font_p01type_tf);
    u8g2.drawStr(64,30,"Soc Peque 4px");
    u8g2.setFont(u8g2_font_open_iconic_check_2x_t);
    u8g2.drawStr(32,60,"ABCDE");
  } while ( u8g2.nextPage() );

  delay(1000);
}
