// ====================================================================================================================
// ====================================================================================================================

 June 19th 2015

// ====================================================================================================================
// ====================================================================================================================

This folder contains various code templates for exporting and creating microcontroller projects directly from within the LED Matrix Studio application. File menu -> Code Templates.

Instead of just exporting the data, it's now possible to export the data in to a preconfigured template - instant code! Instant demo!

Within this folder are two kinds of file:

Code    : <filename>.<extension>
Template: <filename>.<extension>.template

The code file contains the source code (specific to the platform folder), complete with special "tokens" that identify areas that should be filled-in by the software.

The template file contains instructions on how the data should be configured so that it's in a format that source code expects.

If you've created some templates (or would like new tokens), or wish to create them, then please get in touch!!

// ====================================================================================================================
// ====================================================================================================================

Usage:

To populate the template with code and data, use the following tokens:

{$LMS_MATRIX_DATA$}

Inserts the matrix data, based on the .template rules.

{$LMS_FRAMES$}

The number of frames of animation.

{$LMS_BYTES$}

The number of bytes of data (in total).

{$LMS_COUNT$}

The number of entries in the data array.


// ====================================================================================================================
// ====================================================================================================================


Each source code file needs a template to go with it, just append .template to the source code file name. This tells the software how to export the data so it's in the correct format.

.template construction:

{RGB  or {
a: 
b:
c:
d:
e:
f:
g:
h:
i:
r:
v:
w:
y:
z:
}

  {       defines a non-RGB output
  {RGB    will enable RGB output

  a: Export how

  a:0 = Columns
  a:1 = Rows

  b: Output order (Rows OR Columns, depending on selection above)

  b:0 = Top to bottom OR Left to right
  b:1 = Bottom to top OR Right to left
  b:2 = Sure 24x16 special output mode

  c: LSB (least significant bit)

  c:0 = Left
  c:1 = Right

  d: Programming language format

  d:0 = Comma separated
  d:1 = PICAXE EEPROM
  d:2 = C-style (1 dimensional)
  d:3 = C-style (2 dimensional)
  d:4 = Python (1 dimensional)
  d:5 = Python (2 dimensional)
  d:6 = Microchip

  e: Number format

  e:0 = Decimal (base 10)
  e:1 = Binary (base 2)
  e:2 = Hex (base 16)

  f: Number grouping

  f:0 = 8 bits
  f:1 = 16 bits
  f:2 = 32 bits
  f:3 = 8 bits, swap nibbles
  f:4 = 16 bits, swap nibbles
  f:5 = 64 bits
  f:6 = RGB: 8 bits, one byte per colour
  f:7 = RGB: 32 bits

  g: Output order II (in conjunctions with a: and b:)

  g:0 = Left to right OR Top to bottom
  g:1 = Right to left OR Bottom to top
  g:2 - Alternative Top/Bottom OR Alternative Left/Right
  g:3 - Alternative Bottom/Top OR Alternative Right/Left

  h: Output line structure

  h:0 = Row/column
  h:1 = Frame
  h:2 = Bytes

  i: Used for h:2 above

  i:x = where is x is the amount of bytes to output per line

  r: RGB Mode (only use for RGB output)

  r:0 = RGB
  r:1 = BGR
  r:2 = GRB
  
  // == dimension constraints ===============================================

  For some code templates it's highly likely that code expects a matrix of a certain size. These next four parameters
  allow the code template designer to specifiy minimum and maximum matrix sizes.

  For instance, code that outpus a matrix to an 8x8 LED display wouldn't want anything else:

  v:8
  w:8
  y:8
  z:8

  but scrolly message code, across an 8x8 matrix, wouldn't mind how wide the matrix is:

  v:8
  w:0
  y:8
  z:8

  // Use 0 for no limit

  v: Minimum Width

  v:x = Where x is the minimum matrix width allowed by the code template

  w: Maximum width 

  w:x = Where x is the maximum matrix width allowed by the code template

  y: Minimum Height

  y:x = Where x is the minimum matrix height allowed by the code template

  z: Maximum Height

  z:x = Where x is the maximum matrix height allowed by the code template


// ====================================================================================================================
// ====================================================================================================================
