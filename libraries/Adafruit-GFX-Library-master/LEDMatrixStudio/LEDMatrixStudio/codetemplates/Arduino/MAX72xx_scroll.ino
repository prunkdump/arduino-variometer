// ===============================================
// LED Matrix Studio MAX72xx code
//
// Render a scrolly animation across the display
//
// LED Matrix Studio, new single colour, 8 high by any width
//
// Draw your message/pattern on the matrix and export using
// the Export Code option.
//
// See LED_Matrix.leds in the \codetemplates\ folder.
//
// maximumoctopus.com
// maximumoctopus.com/electronics/builder.htm
// 
// Download LedControlMS library from here:
// http://www.instructables.com/id/LED-Matrix-with-Arduino/step2/Arduino-Library/
// 
// ===============================================

#include "LedControlMS.h"

// pin 12 is connected to the DataIn 
// pin 11 is connected to the CLK 
// pin 10 is connected to LOAD / CS
//
LedControl lc = LedControl(12, 11, 10, 1);

unsigned long delaytime = 100;

{$LMS_MATRIX_DATA$}

byte idx = 0;
byte dir = 0;

// ===================================================================

void setup()
{
  lc.shutdown(0, false); // wake the MAX72xx
  lc.setIntensity(0, 8); // Set the brightness to midpoint
  lc.clearDisplay(0);    // clear display
}

void loop()
{ 
  writeToMatrix();
  
  delay(delaytime);
  
  if (dir == 0)
  {
    if (idx != {$LMS_COUNT$} - 8)
    {
      idx++;  
    }
    else
    {
      dir = 1;
    }
  }  
  else
  {
    if (idx != 0)
    {
      idx--;
    }
    else
    {
      dir = 0;
    }
  }
}

void writeToMatrix()
{
  lc.setRow(0, 0, ledarray[idx + 0]);
  lc.setRow(0, 1, ledarray[idx + 1]);
  lc.setRow(0, 2, ledarray[idx + 2]);
  lc.setRow(0, 3, ledarray[idx + 3]);
  lc.setRow(0, 4, ledarray[idx + 4]);
  lc.setRow(0, 5, ledarray[idx + 5]);
  lc.setRow(0, 6, ledarray[idx + 6]);
  lc.setRow(0, 7, ledarray[idx + 7]);  
}
