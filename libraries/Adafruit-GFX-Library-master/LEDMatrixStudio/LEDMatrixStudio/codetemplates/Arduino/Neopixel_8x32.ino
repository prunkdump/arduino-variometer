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
// {$LMS_COUNT$} <- should say 256

#include <Adafruit_NeoPixel.h>

#define PIN 6

{$LMS_MATRIX_DATA$}

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
  strip.show();   // Initialize all pixels to 'off'
  RenderFrame();
}

void loop()
{
  // nothing needed here :(
}

void RenderFrame()
{
  for (int t = 0; t < 256; t++)
  {
    strip.setPixelColor(t, ledarray[t]); 
  }
  
  strip.show();
}
