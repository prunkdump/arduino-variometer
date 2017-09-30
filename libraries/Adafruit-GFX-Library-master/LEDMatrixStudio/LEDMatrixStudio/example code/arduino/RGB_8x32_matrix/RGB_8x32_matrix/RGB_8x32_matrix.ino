// ===============================================
// LED Matrix Studio "NeoPixel" template
//
// Works with many RGB 8x32 displays
// (common to eBay)
//
// Render a single frame of colour data.
// LED Matrix Studio, new RGB, 8x32.
//
// Use the Export Code option
//
// maximumoctopus.com
// maximumoctopus.com/electronics/builder.htm
//
// based on the Adafruit Neopixel demo code
// ===============================================
// 256 <- should say 256

#include <Adafruit_NeoPixel.h>

#define PIN 6

uint32_t ledarray[] = {
                   0x00FE91FF, 0x00FE91EA, 0x00FE91D3, 0x00FF91C1, 0x00FF91A7, 0x00FF9199, 0x00FF9E91, 0x00FFAB91, 
                   0x00FFB691, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00FE91D3, 0x00FE91EA, 
                   0x00FE91D3, 0x00FF91C1, 0x00FF91A7, 0x00FF9199, 0x00000000, 0x00FFAB91, 0x00000000, 0x00FFC391, 
                   0x00FFCE91, 0x00000000, 0x00000000, 0x00FFAB91, 0x00000000, 0x00000000, 0x00FF91A7, 0x00FF91C1, 
                   0x00FF91A7, 0x00FF9199, 0x00FF9E91, 0x00FFAB91, 0x00FFB691, 0x00FFC391, 0x00FFCE91, 0x00FFD891, 
                   0x00FFE091, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00FF9E91, 0x00FF9199, 
                   0x00FF9E91, 0x00FFAB91, 0x00000000, 0x00FFC391, 0x00FFCE91, 0x00FFD891, 0x00000000, 0x00FFE891, 
                   0x00FFF091, 0x00000000, 0x00FFE091, 0x00000000, 0x00000000, 0x00000000, 0x00FFB691, 0x00FFAB91, 
                   0x00FFB691, 0x00FFC391, 0x00FFCE91, 0x00FFD891, 0x00FFE091, 0x00FFE891, 0x00FFF091, 0x00FFF891, 
                   0x00F8FE91, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00FFCE91, 0x00FFC391, 
                   0x00FFCE91, 0x00FFD891, 0x00000000, 0x00FFE891, 0x00000000, 0x00FFF891, 0x00000000, 0x00E3FE91, 
                   0x00CFFE91, 0x00E3FE91, 0x00000000, 0x00FFF891, 0x00000000, 0x00FFE891, 0x00FFE091, 0x00FFD891, 
                   0x00FFE091, 0x00FFE891, 0x00FFF091, 0x00FFF891, 0x00F8FE91, 0x00E3FE91, 0x00CFFE91, 0x00BBFF91, 
                   0x00A7FF91, 0x00BBFF91, 0x00CFFE91, 0x00E3FE91, 0x00F8FE91, 0x00FFF891, 0x00FFF091, 0x00FFE891, 
                   0x00FFF091, 0x00FFF891, 0x00F8FE91, 0x00E3FE91, 0x00CFFE91, 0x00BBFF91, 0x00000000, 0x0092FF91, 
                   0x0091FFA2, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00F8FE91, 0x00FFF891, 
                   0x00F8FE91, 0x00E3FE91, 0x00CFFE91, 0x00BBFF91, 0x00A7FF91, 0x0092FF91, 0x00000000, 0x0091FFB4, 
                   0x0091FFC6, 0x0091FFB4, 0x0091FFA2, 0x0092FF91, 0x00A7FF91, 0x00BBFF91, 0x00CFFE91, 0x00E3FE91, 
                   0x00CFFE91, 0x00BBFF91, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0091FFD7, 
                   0x0091FFE9, 0x00000000, 0x0091FFC6, 0x00000000, 0x0091FFA2, 0x00000000, 0x00A7FF91, 0x00BBFF91, 
                   0x00A7FF91, 0x0092FF91, 0x00000000, 0x0091FFB4, 0x0091FFC6, 0x0091FFD7, 0x00000000, 0x0091FFFA, 
                   0x0091F0FE, 0x0091FFFA, 0x0091FFE9, 0x0091FFD7, 0x0091FFC6, 0x0091FFB4, 0x0091FFA2, 0x0092FF91, 
                   0x0091FFA2, 0x0091FFB4, 0x00000000, 0x0091FFD7, 0x00000000, 0x00000000, 0x00000000, 0x0091DEFE, 
                   0x0091CCFE, 0x00000000, 0x0091F0FE, 0x00000000, 0x0091FFE9, 0x00000000, 0x0091FFC6, 0x0091FFB4, 
                   0x0091FFC6, 0x0091FFD7, 0x00000000, 0x00000000, 0x00000000, 0x0091DEFE, 0x00000000, 0x0091B9FF, 
                   0x0091A7FF, 0x0091B9FF, 0x0091CCFE, 0x0091DEFE, 0x0091F0FE, 0x0091FFFA, 0x0091FFE9, 0x0091FFD7, 
                   0x0091FFE9, 0x0091FFFA, 0x0091F0FE, 0x0091DEFE, 0x0091CCFE, 0x0091B9FF, 0x00000000, 0x009194FF, 
                   0x00A793FF, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0091F0FE, 0x0091FFFA, 
                   0x0091F0FE, 0x0091DEFE, 0x0091CCFE, 0x0091B9FF, 0x0091A7FF, 0x009194FF, 0x00000000, 0x00BC93FF, 
                   0x00D392FF, 0x00BC93FF, 0x00A793FF, 0x009194FF, 0x0091A7FF, 0x0091B9FF, 0x0091CCFE, 0x0091DEFE, 
                   0x0091CCFE, 0x0091B9FF, 0x00000000, 0x009194FF, 0x00000000, 0x00000000, 0x00000000, 0x00E892FF, 
                   0x00FE91FF, 0x00E892FF, 0x00D392FF, 0x00BC93FF, 0x00A793FF, 0x009194FF, 0x0091A7FF, 0x0091B9FF, 
                   };


// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(256, PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
  strip.begin();
  strip.setBrightness(64);
  strip.show();   // Initialize all pixels to 'off'
  
  RenderFrame();
}

void loop()
{
  // nothing needed here :(
  delay(1000);
  
  RenderFrame();
}

void RenderFrame()
{
  for (uint16_t t = 0; t < 256; t++)
  {
    strip.setPixelColor(t, ledarray[t]);
  }
  
  strip.show();
}
