/*

  *** WHAT IS THIS? ***
  * This is an Arduino code for a VFO controller based on AD9833 DDS chip.
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
  * 
  * This sketch is optimised for 30m HAM band.
  * 
  * CALIBRATION PROCEDURE
  * You need a calibrated source at the given "target_freq"
  * that has been chosen to be 10'120'000 Hz.
  * Power everything up and wait for the warm-up time, let's
  * say 15 minutes at room temperature (20-25 °C).
  * Put the circuit in "TX" mode and tune until the measured
  * frequency is 10'120'000 Hz: ignore what is shown on the
  * display. Tuning to 10'120'000 depends on how you compare
  * with the reference.
  * Wait for 20 seconds without touching any control.
  * Power off the VFO circuit.
  * Press the "STEP" button, keep it pressed and power up
  * the VFO circuit. Keep STEP pressed until on the display
  * you read "Calibration OK". 
  * Power cycle once more the VFO (don't press anything) and
  * enjoy your calibrated VFO.
  
  
  * OLD COMMENTS BELOW LEFT FOR DOCUMENTING THE EVOLUTION THAT
  * BROUGHT TO THIS CODE.
 
  This entire program is taken from Jason Mildrum, NT7S and Przemek Sadowski, SQ9NJE.
  There is not enough original code written by AK2b to make it worth mentioning.
  http://nt7s.com/
  http://sq9nje.pl/
  http://ak2b.blogspot.com/
  I made some mods ...first updating the sketch to new library from NT7S
  ..in frequency coverage and the mode for frequency change..
  pressing the encoder and turn it at same time ...it will move a underline showing
  the place where it is OK to change
  THIS SKETCH IS A VFO FOR Direct Conversion equipments or frequency generator
  http://py2ohh.w2c.com.br/

  byZYW 20221217 Works with EtherKit v2.0.4 Si5351 library. Going to update it soon.
  byZYW 20221218 Now using EtherKit 2.1.4; reduces tuning clicks in a receiver

  byZYW 20230226
  this is a sketch for a single HAM band RTX (20m). The step choice has been restricted
  accordingly (it's no use to step by 1 MHz within the same HF band).
  The tuned frequency is saved when the dial is not touched for 10 seconds. There is no
  EEPROM wear reduction routine.

  byZYW 20230726:
  created a fork of the prvious 20m VFO with Si5351 to use an AD9833, with the same UI. 
  This fork handles both break-in delay and an "analog" BFO spanning both sidebands.
  It also generates the sidetone at the same frequency of the BFO. [
*/

#include <Rotary.h>
#include <MD_AD9833.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>   // byZYW



// ALL PIN ALIASES GO HERE, so we don't duplicate
#define ENCODER_A    3                      // Encoder pin A  nano/uno pin D3 - bare 328p pin 5
#define ENCODER_B    2                      // Encoder pin B  nano/uno pin D2 - bare 328p pin 4
#define ENCODER_BTN  A0                     //  - bare 328p pin 23
#define TX_INPUT     A1                   // nano/uno pin A1 - bare 328p pin 24
#define KEY_INPUT    A2                   // nano/uno pin A2 - bare 328p pin 25
#define KEY_OUTPUT   A3                   // nano/uno pin A3 - bare 328p pin 26
#define DELAY_CTRL   A4                   // nano/uno pin A4 - bare 328p pin 27
#define RIT_CTRL     A5                   // nano/uno pin A5 - bare 328p pin 28
#define TONE_OUT     12                    // nano/uno pin D9 - bare 328p pin 18
#define LCD_RS       9                   // LCD pin 4 - nano/uno pin D12 - bare 328p pin 15
#define LCD_E         8                  // LCD pin 6 nano/uno pin D8 - bare 328p pin 14
#define LCD_D4        4                  // LCD pin11 nano/uno pin D4 - bare 328p pin 6
#define LCD_D5        5                  // LCD pin12 nano/uno pin D5 - bare 328p pin 11
#define LCD_D6        6                  // LCD pin13 nano/uno pin D6 - bare 328p pin 12
#define LCD_D7        7                  // LCD pin14 nano/uno pin D7 - bare 328p pin 13
#define DATA  11  ///< SPI Data pin number
#define CLK   13  ///< SPI Clock pin number
#define FSYNC 10  ///< SPI Load pin number (FSYNC in AD9833 usage)

#define RX 0
#define TX 1

#define STEPHIGHLIGHTSYMBOL  0x5E       // symbol on the lower line that marks the current step " ^ "

#define EEFREQADDRESS 2  // byZYW EEPROM address for storing VFO value
#define EECALADDRESS  15 // byZYW EEPROM address for storing calibration value

#define FREQ_LOW_LIMIT  10095000 // in Hz
#define FREQ_HIGH_LIMIT 10155001 // in Hz
#define MAXCALFACTOR        5000 // in Hz  - maximum |cal_factor|
#define SIDETONE             800 // in Hz
#define BFO                 -800 // in Hz


LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);       // LCD - pin assignement in

//MD_AD9833  AD(FSYNC);  // Hardware SPI
MD_AD9833  AD(DATA, CLK, FSYNC);  // My connections

Rotary r = Rotary(ENCODER_A, ENCODER_B);

long vfo = 10100000; //start freq now 10.1MHz expressed in Hz - change to suit   byZYW
const long target_freq  = 10120000; // 10.120000 MHz, used as sample frequency for dynamic calibration
long rit = 0;  //  RIT in ten hertz  
long bfo = BFO; // BFO is 800 Hz. Receiver is set 800 Hz below. Change it to suit your preference

long cal_factor = 300; // CORRECTION in Hz  
/*
 *  ABOUT THE CORRECTION FACTOR.
 *  It is an unsigned int variable, so this value
 *  MUST be a positive Integer, expressed in Hz.
 *  
 *  If this value needs to be SUBSTRACTED to the
 *  DDS frequency to get what is displayed you
 *  *MUST* edit 4 lines below and change '+' into '-'.
 */

volatile uint32_t radix = 100;    //start step size - change to suit

boolean changed_f = 0;
boolean ee_needs_update = 0;
short und = 3;   //controle do underline
short pot = 3;   // controle de multiplicador

#define Direct_conversion //What you see on display is what you get

/**************************************/
/* Interrupt service routine for      */
/* encoder frequency change           */
/**************************************/
ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_CW) {
    set_frequency(1);
  } else if (result == DIR_CCW) {
    set_frequency(-1);
  }
}
/**************************************/
/* Change the frequency and underline  */
/* dir = 1    Increment               */
/* dir = -1   Decrement
  /**************************************/
void set_frequency(short dir)
{
  if (!digitalRead(ENCODER_BTN))  // if the encoder button is pressed, let's move the cursor and compute new pot
  {

    //lcd.setCursor( 12 - und, 0);
    lcd.setCursor(0,1);            // second line
    lcd.print("                "); // clear line
    lcd.setCursor(12 - und, 1);    // position the STEP symbol and
    lcd.write(STEPHIGHLIGHTSYMBOL); // show it
    
/*  Hz  | und
    1     1
    10    2
    100   3
    1000  5
   10000  6
  100000  7
*/
    if (dir == -1) {  // byZYW if direction is anticlockwise
      und += 1;      // move the cursor leftbound
      switch (und) { // handle dots and skip over them
        case 4 :
          und = 5;
          break;
        case 8 :
          und = 7; // cant' go further than this
          break;
//          und = 9;
//          break;
//        case 12 :
//          und = 11;
//          break;
      }

    }

    if (dir == +1)  // byZYW if direction is clockwise move the cursor rightbound
    {
      und += -1;
      switch (und) {
        case 4 :
          und = 3;
          break;
        case 8 :  // byZYW this should never happen with 20m constraint
          und = 7;
          break;
        case 0 :
          und = 1;
          break;
      }
    }

    pot = und;
    if (und > 3) (pot += -1); // reduce the pot value (that chooses radix) to take into account thousand dots
    if (und > 7) (pot += -1);

    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(12 - und, 1);
    lcd.write(STEPHIGHLIGHTSYMBOL);
    

  } else {  // the encoder button is not pressed, we're just retuning not changing the step

    if (dir == 1)
      vfo += radix;

    if (dir == -1) {
      if  (vfo > radix ) {
        vfo -= radix;
      }
    }

    if (vfo >= FREQ_HIGH_LIMIT) { // if we're trying to go past the upper limit, step back one radix
      vfo -= radix;
    }
    
    if (vfo <= FREQ_LOW_LIMIT) {  // if we're trying to go below the lower limit, step up one radix
      vfo += radix;
    }

    changed_f = 1;

  }

}
/**************************************/
/* Read the button with debouncing    */
/**************************************/
boolean get_button()
{
  if (!digitalRead(ENCODER_BTN))

  {

    delay(20);
    if (!digitalRead(ENCODER_BTN))
    {
      while (!digitalRead(ENCODER_BTN));
      return 1;

    }
  }
  return 0;
}

/**************************************/
/* Displays the frequency             */
/**************************************/
void display_frequency()
{
  uint16_t f, g;

//  lcd.clear();
//  delay(1);
  lcd.setCursor(1, 0);

  f = (vfo) / 1000000;    //variable is now vfo instead of 'frequency'
  if (f < 100)  {
    lcd.print(' ');
  }
  if (f < 10) {
    lcd.print(' ');
  }
  lcd.print(f);
  lcd.print('.');
  f = (vfo % 1000000) / 1000;
  if (f < 100)
    lcd.print('0');
  if (f < 10)
    lcd.print('0');
  lcd.print(f);
  lcd.print('.');
  f = vfo % 1000;
  if (f < 100)
    lcd.print('0');
  if (f < 10)
    lcd.print('0');
  lcd.print(f);
  lcd.print(" Hz ");

//  lcd.setCursor(12 - und, 1);
//  lcd.write(STEPHIGHLIGHTSYMBOL);


}


void setup()
{

  long tmp_vfo;   // byZYW

  lcd.begin(16, 2);                                                    // Initialize and clear the LCD
  lcd.clear();

  AD.begin();
  
  // init the AD9833
  AD.setFrequency(MD_AD9833::CHAN_0, vfo);
  AD.setMode(MD_AD9833::MODE_SINE);


  EEPROM.get(EEFREQADDRESS, tmp_vfo);   // byZYW
  if ((tmp_vfo < FREQ_HIGH_LIMIT) && (tmp_vfo > FREQ_LOW_LIMIT)) {  // byZYW  make sure we have a valid value in EEPROM block
    vfo = tmp_vfo;
  } else {
    vfo = (FREQ_LOW_LIMIT + FREQ_HIGH_LIMIT) / 2; // byZYW   set us at the middle of the MIN/MAX range
  }

  pinMode(ENCODER_BTN, INPUT_PULLUP);
  pinMode(TX_INPUT, INPUT_PULLUP);
  pinMode(KEY_INPUT, INPUT_PULLUP);
  pinMode(KEY_OUTPUT, OUTPUT);
  pinMode(DELAY_CTRL, INPUT);
  pinMode(RIT_CTRL, INPUT);
  pinMode(TONE_OUT, OUTPUT);

  digitalWrite(KEY_OUTPUT, LOW); // start in RX mode!!
  digitalWrite(ENCODER_A, HIGH); // uncomment in case the rotary encoder has no pullup resistors
  digitalWrite(ENCODER_B, HIGH); // so use those inside the micro. Note that it might not work and need real resistors!
  PCICR |= (1 << PCIE2);           // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();

  // *** IS IT CALIBRATION TIME? ***
  // If at power on the button in closed, then use the stored frequency
  // to perform soft calibration of the generator, meaning to compute
  // the static correction factor. The correction factor is a constant
  // value in Hz to add/substract to the set frequency so that it matches
  // the display value.
  // Calibration MUST be made at the "target_freq" 

  if (get_button() == 1) {
    cal_factor = vfo - target_freq;
    // let's check the cal_factor falls within a legit range
    // as defined in the MAXCALFACTOR
    if (abs(cal_factor) < MAXCALFACTOR ) {
      // we have a valid cal_factor so we save it for the future
      EEPROM.put(EECALADDRESS, cal_factor);
      lcd.clear();
      delay(1);
      lcd.setCursor(0, 0);
      lcd.print("Cal: ");
      lcd.print(cal_factor);
      lcd.print(" Hz");
      lcd.setCursor(0, 1);
      lcd.print("Calibration OK");
      delay(3);
    } else {
      EEPROM.put(EECALADDRESS, 0);
      lcd.setCursor(0, 0);
      lcd.print("Cal: ");
      lcd.print(cal_factor);
      lcd.print(" Hz !!!");
      lcd.setCursor(0, 1);
      lcd.print("CalibrationRESET");
      cal_factor = 0;     
      delay(3);
    }
  } else {
    // let's read the cal_factor
    EEPROM.get(EECALADDRESS, cal_factor);
    // and check it is within a reasonable value
    if (abs(cal_factor) > MAXCALFACTOR ) {
      lcd.setCursor(0, 0);
      lcd.print("REPEAT Calibration");
      lcd.setCursor(0, 1);
      lcd.print("CalibrationRESET");      
      EEPROM.put(EECALADDRESS, 0);
      cal_factor = 0;
      delay(3);
    }
  }

// 0123456789012345
//  10.125.00 +1200
//        BKIN 1234

// +900   BKIN 740 

#ifdef Direct_conversion
  AD.setFrequency(MD_AD9833::CHAN_0, vfo+(rit*10)+bfo+cal_factor); // RX signal
  AD.setFrequency(MD_AD9833::CHAN_1, vfo+cal_factor); // TX signal  
  AD.setActiveFrequency(MD_AD9833::CHAN_0);  // active in RX
#endif
  
  display_frequency();  // Update the display

}


void loop()
{
  static unsigned long breakin_delay, old_breakin_delay, bkin_difference;
  static unsigned long ee_needs_update_time;
  static unsigned long went_into_tx;
  static long old_rit;  // [10Hz]
  static long sidetone = SIDETONE; // [Hz]
  static unsigned int rtx_status = RX;

  rit = map(analogRead(RIT_CTRL), 0, 1023, -20, 20) * 5;  // rit is in ten Hz, from -1200 Hz to 1200 Hz in steps of 50 Hz
  if (rit != old_rit) {
    old_rit = rit;
    lcd.setCursor(0, 1);
    if (rit == 0) {
      lcd.print("0     ");
    } else {
      if (rit > 0) {
        lcd.print("+");
      } else {
        lcd.print("-");
      }
  
      lcd.print(abs(rit)); // max 3 chars
      lcd.print("0   ");
    }

    // since the RIT has changed, we have to update where we are receiving
    if ((vfo < FREQ_HIGH_LIMIT) && (vfo > FREQ_LOW_LIMIT)) {  // if we have not reached the band limit apply the change
      AD.setFrequency(MD_AD9833::CHAN_0, vfo+(rit*10)+bfo+cal_factor); // RX signal
    }
      
  }
  
  breakin_delay = map(analogRead(DELAY_CTRL), 0, 1023, 40, 400) * 10;  // in milliseconds from 400 ms to 4000 ms in steps of 10 ms
  if (breakin_delay != old_breakin_delay) {
    old_breakin_delay = breakin_delay;
    lcd.setCursor(6, 1);
    lcd.print(" BKIN "); // 6 char
    lcd.print(breakin_delay); // max 4 char
    lcd.print(" ");
  }

    if (digitalRead(KEY_INPUT) == 1) {            // byZYW || if the /KEY_INPUT is high we're in RX mode

      noTone(TONE_OUT);  // on key up stop the sidetone in any case

      
      if (rtx_status == TX) {
          bkin_difference = millis() - went_into_tx;        
          if (bkin_difference > breakin_delay) { // breakin timeout, return in RX on RX frequency
            AD.setActiveFrequency(MD_AD9833::CHAN_0);
            AD.setMode(MD_AD9833::MODE_SINE);
            digitalWrite(KEY_OUTPUT, LOW);
            rtx_status = RX;
          } else {
            AD.setMode(MD_AD9833::MODE_OFF);
          }
      } else {
        lcd.setCursor(0,0);
        lcd.print("R");
      }

    } else {                                // else we are in TX mode, key is down

      if (rtx_status == RX) {
          rtx_status = TX;
          lcd.setCursor(0,0);
          lcd.print("T");
          AD.setActiveFrequency(MD_AD9833::CHAN_1);
      }
      AD.setMode(MD_AD9833::MODE_SINE);
      went_into_tx = millis();  // retrigger the bk-in timer
      tone(TONE_OUT, sidetone);
      digitalWrite(KEY_OUTPUT, HIGH);
      
    }

  
    // Update the display if the frequency has been changed
    // and also update the EEPROM  byZYW
    if (changed_f)
    {
      display_frequency();
  
      if ((vfo < FREQ_HIGH_LIMIT) && (vfo > FREQ_LOW_LIMIT)) {  // if we have not reached the band limit apply the change
        AD.setFrequency(MD_AD9833::CHAN_0, vfo+(rit*10)+bfo+cal_factor); // RX signal
        AD.setFrequency(MD_AD9833::CHAN_1, vfo+cal_factor); // TX signal  
      }

      lcd.setCursor(0, 1);
      if (rit == 0) {
        lcd.print(" 0    ");
      } else {
        if (rit > 0) {
          lcd.print("+");
        } else {
          lcd.print("-");
        }
    
        lcd.print(abs(rit)); // max 3 chars
        lcd.print("0  ");
      }
      
      lcd.setCursor(6, 1);
      lcd.print(" BKIN "); // 6 char
      lcd.print(breakin_delay); // max 4 char
      lcd.print(" ");

      changed_f = 0;
      ee_needs_update = 1;
      ee_needs_update_time = millis();
    
    }
  
    if (ee_needs_update == 1) { // byZYW
      if ((millis() - ee_needs_update_time) > 10000) { // byZYW update EEPROM only after 10s of inactivity
        EEPROM.put(EEFREQADDRESS, vfo);   // byZYW
        ee_needs_update = 0;
      }
    }
  
  
    if (get_button())
    {
      switch (pot)
      {
        case 1:
          radix = 1;
          break;
        case 2:
          radix = 10;
          break;
        case 3:
          radix = 100;
          break;
        case 4:
          radix = 1000;
          break;
        case 5:
          radix = 10000;
          break;
        case 6:
          radix = 100000;
          break;
        case 7:
          radix = 1000000;
          break;
        case 8:
          radix = 10000000;
          break;
        case 9:
          radix = 100000000;
          break;
  
      }
  
    }

}
