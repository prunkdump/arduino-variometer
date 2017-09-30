#include <SPI.h>
#include <ILI9341_t3.h>
#include <font_LiberationMonoBold.h>
#include <font_LiberationMonoBoldItalic.h>
#include <font_LiberationMono.h>
#include <font_LiberationMonoItalic.h>

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

  int xx = 0;
  tft.setFont(LiberationMono_24);
  tft.setCursor(xx, 4);
  tft.print("Liberation Mono");
  tft.setCursor(xx, 61);
  tft.setFont(LiberationMono_24_Bold);
  tft.print("Liberation Mono");
  tft.setCursor(xx, 122);
  tft.setFont(LiberationMono_24_Italic);
  tft.print("Liberation Mono");
  tft.setCursor(xx, 183);
  tft.setFont(LiberationMono_24_Bold_Italic);
  tft.print("Liberation Mono");
}

void loop(void) {
}

