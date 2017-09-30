#include <SPI.h>
#include <ILI9341_t3.h>
#include <font_Nunito.h>
#include <font_NunitoBold.h>
#include <font_NunitoLight.h>

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
  tft.setFont(Nunito_40);
  tft.setCursor(xx, 4);
  tft.print("Nunito");
  tft.setCursor(xx, 74);
  tft.setFont(Nunito_40_Bold);
  tft.print("Nunito Bold");
  tft.setCursor(xx, 144);
  tft.setFont(Nunito_40_Light);
  tft.print("Nunito Light");
}

void loop(void) {
}

