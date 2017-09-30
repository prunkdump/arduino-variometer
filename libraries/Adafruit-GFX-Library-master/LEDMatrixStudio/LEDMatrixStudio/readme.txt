 ===============================================================================
 =                                                                             =
 =  LED Matrix Studio v0.8.8 BETA                                              =
 =                                                                             =
 =  August 17th 2017                                                           =
 =                                                                             =
 =  www.freshney.org // paul@freshney.org                                      =
 =                                                                             =
 =  www.twitter.com/maximumoctopus                                             =
 =                                                                             =
 =  www.maximumoctopus.com/electronics/builder.htm                             =
 =                                                                             =
 =  Application:                                                               =
 =  www.maximumoctopus.com/electronics/downloads/LEDMatrixStudio.zip           =
 =                                                                             =
 =  Source code:                                                               =
 =  www.maximumoctopus.com/electronics/downloads/LEDMatrixStudioSource.zip     =
 =  https://sourceforge.net/projects/led-matrix-studio/                        =
 =                                                                             =
 =  Facebook                                                                   =
 =  https://www.facebook.com/LEDMatrixStudio                                   =
 =                                                                             =
 ===============================================================================

 Hello!
 
 Thanks for downloading the latest version of my LED Matrix Studio.

 A few years ago I started playing with Picaxe and Arduino 
 microcontrollers and one of the first things I bought was an 8x8
 LED matrix. In the passing couple of years I've bought lots and lots
 of LED boards! I love LEDs, I think it's an addiction!

 After working out the bit-patterns on a piece of paper a couple of
 times I did what any programmer would do - I wrote a simple 
 program to work out the bit-patterns for me. That simple program
 morphed in to three separate programs, each designed for slightly 
 different things. 

 On the 10th of June 2012 I decided to start from scratch and
 build an all-purpose application with every feature I (and others)
 could ever want.
 
 I've designed it to be as easy to use as possible. Click New and select
 the size and type of matrix you need.

 A simple column and row data display are available at the bottom
 of the display, but select Export from the Project menu to take
 advantage of the software's export options. 
 
 Incidentally, if you think there is an option missing then please email
 me and I'll make sure it gets added to the next release.
 
 There is a special Sure Electronics 24x16 mode that outputs the column
 data in the order that the device requires. This is one of my favourite
 matrix devices, hence why I have this mode!
 
 Please email me if you have any special requests for other devices. 

 Features:
 
 - Supports up to 128x128 matrix
 - Supports single colour, bi-colour and RGB matrices
 - Seven sizes of "pixel" allows for use on almost any resolution PC
   Plus auto-size!
 - Square or round "pixels"
 - Three brush sizes (1 pixel, 2x2 and 3x3 pixels)
 - Export data in all possible combinations!
 - Export as code or binary
 - Select either binary, decimal or hex output (with $ or 0x prefix)
 - Select either normal, curly or square brackets
 - Many different draw modes
     simple click to toggle on/off (left mouse button)
     Freehand draw mode (right mouse button)
     Filled rectangles
     Empty rectangles
	 Empty circle
     Line from A->B
	 Multi-draw (all frames simultaneously)
     Text (using customisable fonts, two included 5x7 and 3x5)
	 Random
	 Gradient (draw using the middle button)
 - Flip, mirror, invert, rotate, scroll a matrix
 - Animation support, unlimited number of frames
 - Font designer mode (single colour or RGB)
 - Unlimited per-frame undo/redo
 - Save/Load native format
 - Unlimited "presets", predefined sizes and formats
 - Export animation or single frame
 - 10 separate user buffers/scratchpads
 - Auto-save option
 - Import from bitmap image (one or more frames)
 - Export to bitmap 
 - Preview mode, view the image at x1, x2, x3, x4, x5 and x6 pixel size.
 - Open source, download from the address at the top of this doc

 If you find a bug or have a feature request then *please* email me.
 
 Look in the /help/ folder for more information.
 
 Many thanks,

 Paul A Freshney (paul@freshney.org)

========================================================================

 Credits:

   All coding       : Paul A Freshney
   Development Cats : Rutherford and Freeman
   Thanks           : Lorenz, Greg, Andrew, Apostolos, David, Peter,
                      Zoltan, Gary and Steve Turner

 Help wanted!

 Have you created a fantastic animation, font, graphic or a preset
 for an LED matrix device?

 Please consider sending it to me so that I may include it with
 future updates!

========================================================================
========================================================================

========================================================================
== Updates for 0.8.8 beta ==============================================
========================================================================

- Fixed an RGB copy/paste bug
- Fixed couple of minor Preview display bugs

========================================================================

========================================================================
== Updates for 0.8.7 beta ==============================================
========================================================================

- Fixed a bug where drawing in RGB mode with a horizontal gradient 
  wouldn't work properly.
- Fixed a bug where drawing shapes could crash in RGB mode :(
- Tidied up the source code a bit :)

========================================================================
== Updates for 0.8.6 beta ==============================================
========================================================================

- Added binary export option (raw data, no formatting of any kind)
- Added full frame-based history (undo and redo) with infinite levels!
- Fixed an output bug (RGB column mode)
- Added drop down list of popular options to Frame Count selector on
  "New Project" window
- Added Preview details to settings (load and save on startup)
- Other minor tweaks

========================================================================
== Updates for 0.8.5 beta ==============================================
========================================================================

- Fixed a bug with circle draw modes
- Fixed a bug which caused the "middle mouse button" to appear in
  single colour mode

========================================================================
== Updates for 0.8.4 beta ==============================================
========================================================================

- Added Gradient mode reminder graphic, shows that middle mouse button
  draws gradient
- Fixed unchecked "Clear All" setting in new project not doing anything
- Other minor fixes and tweaks

========================================================================
== Updates for 0.8.3 beta ==============================================
========================================================================

- Added 128x128 pixel matrix support (BETA!)
- Couple minor fixes and tweaks (rotate any angle + others)

========================================================================
== Updates for 0.8.2 beta ==============================================
========================================================================

- Added filled circle draw mode
- Other minor fixes

========================================================================
== Updates for 0.8.1 beta ==============================================
========================================================================

- Added RGB font support
- Removed animation frame limit! Add as many as you like
- Improved memory usage
- Added horizontal gradient mode

========================================================================
== Updates for 0.8.0 beta ==============================================
========================================================================

- Fixed Presets not appearing/working
- Fixed Bi-colour mode
- Added RGB mode
- Added Code Templates (see doc in folder)
- Added Randomness selection to random drawing mode (RGB only!)
- Added "dead pixels", pixels that can't be drawn on, or exported
- Added new "Alternate up/down" modes for display boards that need it
  (various 8x32 RGB boards on eBay)
- Added optimise option. Works well on RGB or 16/32 bit outputs. Capable
  of giving good compression of simple-ish data with almost no extra
  microcontroller overhead.  
- Plus lots of other tweaks and fixes

========================================================================
== Updates for 0.7.16 ==================================================
========================================================================

- Complete rewrite of the matrix rendering engine
  (everyone should see a performance increase!)
- Drawing, freehand/tools now follow standard conventions 
  Left click = "ON", right click = "OFF"
- New/Open/Save buttons on top toolbar
- New/Open/Save logic is much better (more logical)
- Added a new output option "Microchip" (dt ...... ; comment)
- Added "Are you sure you want to quit?" message
- Added "Comment" field for each matrix (Edit -> Edit Comment)
  (is included with exported data)
- Added circle drawing tool
- Added multi-draw tool (draws on every frame simultaneously!)
- Added new brush sizes, 2x2 and 3x3 pixels
- Added "Auto" pixel size mode
- Added a font viewer (View->Font Viewer)
- Drawing tools now update in realtime
- Copy shows bounding box
- Lots of little fixes and improvements

========================================================================
== Updates for 0.7.15 ==================================================
========================================================================

- Fixed a bug with export options profiles not loading properly
- Fixed a bug with export options "binary output" mode
- Added more details to export options output
- Added x4 preview mode
- Added Donate button :0

========================================================================
== Updates for 0.7.14 ==================================================
========================================================================

- Fixed a bug in the Font Mode data loader
- Added a preview of the current matrix: x1, x2 and x3 pixel magnification

========================================================================
== Updates for 0.7.13 ==================================================
========================================================================

- Fixed the loader :(

========================================================================
== Updates for 0.7.12 ==================================================
========================================================================

- Much improved export functionality
- User memories can be exported in the same way as normal matrix data
- Removed separate Row/Column toolbars, replaced with a single toolbar
- Fixed a couple of minor bugs

========================================================================
== Updates for 0.7.11b =================================================
========================================================================

- Fixed a bug with row display showing MSB/LSB wrong way around
- Fixed a number of bugs with export
- Added a couple of Python export formats
- Changed the default start char of font mode from 33 to 32

========================================================================
== Updates for 0.7.10 ==================================================
========================================================================

- Fixed a bug that would multiply the amount of data when saving
  animations with some settings
- Added "combine nibbles" option, useful for displays with a width or
  length of 4 pixels (like the Orion4x4 grid board)
- Hex and bracket option settings are saved in each animation file now
- Added option to use above when loading or use the current application
  settings (View->Use format embedded in save files)
- Added ability to change the start ASCII code in font mode
  (default is 33)

========================================================================
== Updates for 0.7.9d ==================================================
========================================================================

- Fix when editing text in column/row boxes with 0x prefix

========================================================================
== Updates for 0.7.9c ==================================================
========================================================================

- Couple of minor fixes

========================================================================
== Updates for 0.7.9b ==================================================
========================================================================

- Fixed a bug which wouldn't select no hex prefix
- Changed a couple of button/control tooltips

========================================================================
== Updates for 0.7.9 ===================================================
========================================================================

- Added option of circular or square pixels

========================================================================
== Updates for 0.7.8 ===================================================
========================================================================

- Fixed a bug that caused the grid to hide behind the top tool bars on
  systems using large fonts

========================================================================
== Updates for 0.7.7 ===================================================
========================================================================

- Added a "gradient" mode for bicolour matrices
- Added a "random" colour drawing mode for bicolour matrices
- Minor bug fixes etc.

========================================================================
== Updates for 0.7.6b ==================================================
========================================================================

- Fixed bug that stopped some files loading :(

========================================================================
== Updates for 0.7.6 ===================================================
========================================================================

- Added bicolour matrix support
- Added optimise mode (beta at moment)
- Added mousewheel up/down to select frame
- Minor modifications to the GUI and other minor updates
- Fixed a few minor bugs

========================================================================
== Updates for 0.7.5 ===================================================
========================================================================

- Added Auto Save option (2/5/10 minute intervals)
  Saves to \saves\_autosave.leds
- Added new icon!
- Added import from bitmap image option
- Increased max frame limit to 500
- Increased max frame size to 64x64
- Column and Row "data boxes" now accept hex values ($ and 0x)
- Fixed a couple of minor bugs

========================================================================
== Updates for 0.7.4 ===================================================
========================================================================

- Added Padding option for hex values
- Fixed a couple of minor bugs (load/save location is now remembered)
- Added option to toggle matrix grid

========================================================================
== Updates for 0.7.3 ===================================================
========================================================================

- Fixed a bug which outputs the 24x16 data incorrectly
- Fixed a bug which makes the hex output settings not correctly
  set on startup
- Fixed a bug which stops the update checker connecting to my website
  (for some reason it's become very picky over agent strings?!)

========================================================================
== Updates for 0.7.2 ===================================================
========================================================================

- Fixed a few minor bugs
- Added flip/invert/mirror all frames option on menu
- Added import option. Imports a single frame from a saved matrix to the
  currently selected frame.
- Increased frame limit to 200.
- Other minor tweaks!


========================================================================
== Updates for 0.7.1 ===================================================
========================================================================

- Fixed a few minor bugs
- Added rotate by any angle feature. Well, actually multiples of 5'.
  Select the angle, and number of frames and the software will generate
  them for you. Each new frame being x degrees further rotated than
  the preceeding frame. Works best with square grids...
- Now shows which user buffers have content by the appearance of a little
  icon beside the menu item
- Customisable animation playback speed, press the right mouse button
  over the play button.
- Other minor tweaks!

========================================================================
========================================================================