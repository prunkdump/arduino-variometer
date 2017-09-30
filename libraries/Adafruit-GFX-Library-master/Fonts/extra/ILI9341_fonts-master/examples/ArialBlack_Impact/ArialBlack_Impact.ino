#include <SPI.h>
#include <ILI9341_t3.h>
#include <font_ArialBlack.h>
#include <font_Impact.h>

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

  tft.setFont(ArialBlack_48);
  tft.setCursor(40, 0);
  tft.println("Arial");
  tft.setCursor(40, 65);
  tft.println("Black");
  tft.setCursor(40, 160);
  tft.setFont(Impact_48);
  tft.printf("Impact");
}

void loop(void) {
}

