# VFO_AD9833_RITstyle_byZYW
A VFO controller for a monoband direct-conversion HAM RTX on 30 m band with AD9833 DDS. 

## NOTE 2023-09-29
I wrote this code adapting a similar Arduino sketch for Si5331 for a friend that builds simple direct-conversion amateur radio transceivers. I know there is no schematic diagram, but the information needed to draw it is in the comments inside the code. Be aware that given "the mess" with Arduino libraries, you might be compiling with a different version/author and something might break, or compile OK and then misbehave when running.
This repo is primarily for personal backup. Use the code as inspiration and guidance in writing your own.

 ## WHAT IS THIS?
 
   This is an Arduino code for a VFO controller based on AD9833 DDS chip.
   With this circuit you can control a direct-conversion amateur radio
   transceiver on the 30 metres band.
   Tuning is via a rotary encoder and you need a N.O. button to change the
   tuning step. All info is displayed on a 16x2 HD44780 LCD.
   There is an analog control for a RIT (+/-1200 Hz from RX frequency) and
   an analog control for the break-in delay (400 to 4000 ms). You can adjust
   these settings deep down into the code.
   There is also a calibration procedure that retains the value in the microcontroller
   EEPROM.
   
   The display shows:
  * 1st line: RX/TX status  TX carrier frequency
  * 2nd line: RIT value in Hz   Break-in delay in ms
   

Credits for the original code -that was for another DDS chip - probably go to
"This entire program is taken from Jason Mildrum, NT7S and Przemek Sadowski, SQ9NJE.
  There is not enough original code written by AK2b to make it worth mentioning.
  [cut]
  and PY2OHH"

## AD9833
The AD9833 DDS chip has an upper limit of 12.5 MHz. Using it on 10 MHz is quite a stretch but it works. The output must be low-pass filtered, but you know that!
