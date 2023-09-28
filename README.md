# VFO_AD9833_RITstyle_byZYW
A VFO controller for a monoband direct-conversion HAM RTX on 30 m band with AD9833 DDS. 

 *** WHAT IS THIS? ***
  * This is an Arduino code for a VFO controller based on AD9833 DDS chip.
  * With this circuit you can control a direct-conversion amateur radio
  * transceiver on the 30 metres band.
  * Tuning is via a rotary encoder and you need a N.O. button to change the
  * tuning step. All info is displayed on a 16x2 HD44780 LCD.
  * There is an analog control for a RIT (+/-1200 Hz from RX frequency) and
  * an analog control for the break-in delay (400 to 4000 ms). You can adjust
  * these settings deep down into the code.
  * There is also a calibration procedure that retains the value in the microcontroller
  * EEPROM.
  * 
  * The display shows:
  * 1st line: RX/TX status  TX carrier frequency
  * 2nd line: RIT value in Hz   Break-in delay in ms
  * 

Credits for the original code -that was for another DDS chip - probably go to
"This entire program is taken from Jason Mildrum, NT7S and Przemek Sadowski, SQ9NJE.
  There is not enough original code written by AK2b to make it worth mentioning.
  [cut]
  and PY2OHH"
