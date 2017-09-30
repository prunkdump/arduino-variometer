#include <SPI.h>
#include <ILI9341_t3.h>
#include <font_DroidSans.h>
#include <font_DroidSans_Bold.h>
#include <font_DroidSansMono.h>

#define TFT_DC  9
#define TFT_CS 10

// Use hardware SPI (#13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

void setup() {
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  //tft.setTextSize(2);
  //tft.println("Waiting for Arduino Serial Monitor...");

  Serial.begin(9600);
  //while (!Serial) ; // wait for Arduino Serial Monitor
  //tft.fillScreen(ILI9341_BLACK);
  Serial.println("ILI9341 Test!"); 

  int xx = 30;
  tft.setFont(DroidSans_32);
  tft.setCursor(xx, 4);
  tft.print("Droid Sans");
  tft.setCursor(xx, 60);
  tft.setFont(DroidSans_32_Bold);
  tft.print("Droid Sans");
  tft.setCursor(xx, 100);
  tft.print("Bold");
  tft.setCursor(xx, 150);
  tft.setFont(DroidSansMono_32);
  tft.print("Droid Sans");
  tft.setCursor(xx, 190);
  tft.print("Mono");
}

void loop(void) {
}

