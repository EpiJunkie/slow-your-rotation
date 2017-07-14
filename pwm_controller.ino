///////////////////////////////////////////////////////////////////////////////////////
// Title:         PWM Fan Controller for SuperMicro DAS
// Purpose:       Control 7 PWM fans in case with button to select different fan speed
//                 presets. Give audiable indication to the mode and blink button LED.
//                Monitor temperature sensors
//                
// Changes:       v.01 - Created base script to include modulation of the fans via PWM,
//                        Temperature read out to serial via I2C,
//                v.02 - Changed code to use interupt for button push to trigger a 
//                        change in the fan mode, this is to leave the loop() function 
//                        to display output to serial and in the future vary the speed of 
//                        the fans based on temperature. Added StartupSequence function
//                        to ensure the LEDs are working, plays some sounds, and then
//                        spin up the fans to max speed, then drop the fans back down.
//                v.03 - Documentation up to this point.
//                v.04 - Varying fan speed
//                v.05 - LED integration
//                v.06 - Revising fan handling to use counters. Also to write those changes
//                        to the pins.
//                v.07 - Improvements to code layout, struture
//                v.08 - Minor improvements to code
//                v.09 - Public release, needs revisions to make more readable and also
//                        need to remove references to unwritten functionality.
//                
//                
// Original title: 4-pin PWM Fan Controller
// Original Author: Sky
// Original Creation date: 2015-1-1
//
// Title: 4-pin PWM Fan Controller
// Author: Justin D Holcomb
// Modification date: 2016-3-1
// Related article: http://blog.epijunkie.com/2016/03/pwm-fan-controller-with-arduino/
//
///////////////////////////////////////////////////////////////////////////////////////

// Trinket Pro notes
// Pins 2 and 7 are inuse by the USB
// The bootloader on the Pro Trinket use 4KB of FLASH so the maximum sketch size is 28,672 bytes. 

//// Libraries
// Temperature
#include <Wire.h>
#include "Adafruit_MCP9808.h"
// NeoPixel
#include <Adafruit_NeoPixel.h>

// Create the MCP9808 temperature sensor objects
Adafruit_MCP9808 front_left_temp = Adafruit_MCP9808();
Adafruit_MCP9808 front_right_temp = Adafruit_MCP9808();
Adafruit_MCP9808 board_temp = Adafruit_MCP9808();
Adafruit_MCP9808 psu_temp = Adafruit_MCP9808();

// The default address is 0x18 and the address can be calculated by 'adding' the A0/A1/A2 to the base of 0x18 
// A0 sets the lowest bit with a value of 1, A1 sets the middle bit with a value of 2 and A2 sets the high
//  bit with a value of 4. The final address is 0x18 + A2 + A1 + A0. 
// So for example if A2 is tied to VDD and A0 is tied to VDD, the address is 0x18 + 4 + 1 = 0x1D. 
// If only A0 is tied to VDD, the address is 0x18 + 1 = 0x19
// If only A1 is tied to VDD, the address is 0x18 + 2 = 0x1A
// If only A2 is tied to VDD, the address is 0x18 + 4 = 0x1C


// Defines the PWM pin to use for the Neo Pixel Strip
#define NeoStickPIN 10

// Total natural count of pixels off of PWM pin above
#define LEDtotal 16

// Declares 'strip' as an object for the NeoPixel manipulations
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDtotal, NeoStickPIN, NEO_GRB + NEO_KHZ800);

uint32_t KNcolorToDisplay = strip.Color(64, 64, 64);
uint32_t KNtrailcolorToDisplay = strip.Color(5, 5, 5);
uint32_t color_red = strip.Color(255, 0, 0);
uint32_t color_white = strip.Color(255, 255, 255);
uint32_t color_blue = strip.Color(0, 0, 255);

int KnightRiderPixelBegin = 8; // This references the index number which starts at zero
int KnightRiderPixelEnd = 15; //This references the index number which starts at zero
int UserMode = 0;
int AdaptiveMode = 1;
int StatusColor = 1;
int BPcycleCount = 1; // For button push cycle count - unrelated to cycleCount variable.
uint32_t UMcolorToDisplay = strip.Color(12, 12, 12);
uint32_t AMcolorToDisplay = strip.Color(5, 0, 0);


// Pin Layout
// Pins 2 and 7 are inuse by the USB on the Trinket
//
// Pin 9  PWM    - fan_banks
// Pin 10 PWM    - NeoPixel Stick
// Pin 11 PWM    - 
// Pin 12        - speed_psu_fan
// Pin 13        - speed_bank1a
// Ar            - 
// A0            - 
// A1            - speakerPin
// A2            - 
// A3            - 
// A4 SDA        - I2C Bus
// A5 SCL        - I2C Bus

// A6            - Header not soldered
// A7            - Header not soldered

// Pin 8         - 
// Pin 6  PWM    - ringPin
// Pin 5  PWM    - psu_fan
// Pin 4         - 
// Pin 3  PWM    - buttonPin - Interupt 1
// Pin 1         - 
// Pin 0         - 


// NeoStick Sticks
//
// Pins 1-4 get set at the mode select function. These colors will all match but may differ between modes.
// 1 Indicate Modes 1 - 4
// 2 Indicate Modes 2 - 4
// 3 Indicate Modes 3 - 4
// 4 Indicate Mode 4
// Pins 5-8 get changed each iteration of loop left to right. This will range the color. This will be a ghost rider styled effect.
// 5 Indicate running
// 6 Indicate running
// 7 Indicate running
// 8 Indicate running
//
// Number of LEDs will indicate the fan speed based on a range that makes sense. Pin 16 will only indicate above 240.
// Red color will indicate an increase in speed
// Light blue color will indicate netural
// Dark Blue will indicate cooling
// 9  Indates fan speed
// 10 
// 11 
// 12 
// 13 
// 14 
// 15 
// 16 

#define TicksPerMinute 25 // 1 minute = ~25 ticks
int HighTempTicks;
int MinutesForHighTempReset = 120; 
int HighTempThresholdInMinutes = 25;
int MinutesForFanIncrease = 3;
int MinutesForFanDecrease = 5;
int FanSpeedIncrement = 15;
int MaxTempThreshold = 80;
int OptimalTempThreshold = 76;
int milliSecondsToSpinForPurge = 6000;
int cycleCount = (TicksPerMinute * MinutesForFanIncrease); // Start counter ready for fan adjustment if needed

// No or Light Load
int Mode1psuFanSpeed = 50;
int Mode1bankFanSpeed = 40;
// Common Mode
int Mode2psuFanSpeed = 100;
int Mode2bankFanSpeed = 80;
// Intense Load
int Mode3psuFanSpeed = 150;
int Mode3bankFanSpeed = 120;
// MAX - largely for testing
int Mode4psuFanSpeed = 255;
int Mode4bankFanSpeed = 255;

// Fan Speeds
int psuFanSpeed;
int bankFanSpeed;
int psuFanMINSpeed;
int bankFanMINSpeed;

// Default Brightness for LED Ring
#define ringBrightness 64

// Pin A4 - SDA
// Pin A5 - SCL

// Button Input Pin
#define buttonPin 3

// LED Ring Pin (PWM)
#define ringPin 5

// PWM Pins
#define psu_fan 6
#define fan_banks 9

// Speaker Pin
//int speakerPin = A1;

// Speed Sense Pins
//int speed_psu_fan = 12;
//int speed_bank1a = 13;
//int speed_bank2a = 6;
//int speed_bank3a = 9;
//int speed_bank1b = 5;
//int speed_bank2b = 6;
//int speed_bank3b = 9;

float temp_c_board;
float temp_f_board;


void setup() {

  // Initialize NeoPixel sticks and turn off LEDs.
  strip.begin();
  strip.show();

  // Attach interupt for push button
  //attachInterrupt(1, changeFanSpeedMode, HIGH);
  
  // led ring output
  pinMode(ringPin, OUTPUT);

  //button input
  pinMode(buttonPin, INPUT);

  //pwm pins
  pinMode(psu_fan, OUTPUT);
  pinMode(fan_banks, OUTPUT);

  //Serial.begin(9600);

//  if (!front_left_temp.begin(0x1A)) {
//    Serial.println("Couldn't find front_left_temp sensor!");
//    while (1);
//  }

//  if (!front_right_temp.begin(0x19)) {
//    Serial.println("Couldn't find front_right_temp temp sensor!");
//    while (1);
//  }

// Make sure the sensor is found, you can also pass in a different i2c
// address with tempsensor.begin(0x19) for example
if (!board_temp.begin()) {
  //Serial.println("Couldn't find rear psu side temp sensor!");
  while (1);
}

//  if (!psu_temp.begin(0x18)) {
//    Serial.println("Couldn't find psu_temp sensor!");
//    while (1);
//  }

  StartupSequence();
 
}


void loop() {

  cycleCount++;

// Sets the color for knight rider scroll
// Indicates fan speed increasing
  if (StatusColor==2) {
    KNcolorToDisplay = strip.Color(16, 0, 0);
    KNtrailcolorToDisplay = strip.Color(1, 0, 0);  
// Indicates fan speed decreasing
  } else if (StatusColor==0) {
    KNcolorToDisplay = strip.Color(0, 3, 16);
    KNtrailcolorToDisplay = strip.Color(0, 0, 1);
// Indciates neutral
  } else { 
    KNcolorToDisplay = strip.Color(4, 4, 4);
    KNtrailcolorToDisplay = strip.Color(1, 1, 1);
  }

// Poll Tempature
    board_temp.shutdown_wake(0);   // Don't remove this line! required before reading temp
    // Read and print out the temperature, then convert to *F
    temp_c_board = board_temp.readTempC();
    temp_f_board = temp_c_board * 9.0 / 5.0 + 32;  
    //delay(250);
    //board_temp.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere

// Serial output for debugging
//SerialDebug();

// Check to see if fan changes are needed based on temperature readings
CheckIfFanSpeedChangesNeeded();

// This if statment checks to see if an adaptive mode has been running for a long time.
// If so it does a short fan burst to purge out the hot air.
if (HighTempTicks > ( TicksPerMinute * HighTempThresholdInMinutes )) {
  MAXFanSpinToPurgeHotAir();
}


// Sets adaptive mode based on ranges. Minimum ranges do not need to be defined.
  if (bankFanSpeed == 255 && UserMode != 4 ) {
    AdaptiveMode = 8;
  } else if (bankFanSpeed >= (Mode3bankFanSpeed + (FanSpeedIncrement *4)) && UserMode < 4) {
    AdaptiveMode = 7;
  } else if (bankFanSpeed >= (Mode3bankFanSpeed + (FanSpeedIncrement *3)) && UserMode < 4) {
    AdaptiveMode = 6;
  } else if (bankFanSpeed >= (Mode3bankFanSpeed + (FanSpeedIncrement *2)) && UserMode < 4) {
    AdaptiveMode = 5;
  } else if (bankFanSpeed > Mode3bankFanSpeed || UserMode == 4) {
    AdaptiveMode = 4;
  } else if (bankFanSpeed >= Mode3bankFanSpeed && UserMode < 2) {
    AdaptiveMode = 3;
  } else if (bankFanSpeed >= Mode2bankFanSpeed && UserMode < 1) {
    AdaptiveMode = 2;
  } else if (bankFanSpeed >= Mode1bankFanSpeed && UserMode <= 1) {
    AdaptiveMode = 1;
  } else {
    AdaptiveMode = UserMode;
  }

// Reads if button is pushed and increments mode if it is.
  if ( digitalRead(buttonPin) == HIGH ) {
    UserMode++;

    // Increment the fan mode
    if ( UserMode == 4 ) {
      // MAX
      psuFanSpeed = Mode4psuFanSpeed;
      bankFanSpeed = Mode4bankFanSpeed;
    } else if ( UserMode == 3 ) {
      // Scrub Speed
      psuFanSpeed = Mode3psuFanSpeed;
      bankFanSpeed = Mode3bankFanSpeed;
    } else if ( UserMode == 2 ) {
      // Optimal
      psuFanSpeed = Mode2psuFanSpeed;
      bankFanSpeed = Mode2bankFanSpeed;
    } else {
      // Silent
      UserMode = 1;
      psuFanSpeed = Mode1psuFanSpeed;
      bankFanSpeed = Mode1bankFanSpeed;
    } 

// Set minimum fan speeds for mode
    psuFanMINSpeed = psuFanSpeed;
    bankFanMINSpeed = bankFanSpeed;

// Write the changes to the PWM pins
    analogWrite(psu_fan, psuFanSpeed);
    analogWrite(fan_banks, bankFanSpeed);

// Blink led ring
    for ( int i = 0; i <= UserMode; i++ ) {
      analogWrite(ringPin, 0);
      delay(200);
      analogWrite(ringPin, 255);
      delay(200);
    }

// turn led ring back to original state so it's not too bright
    analogWrite(ringPin, ringBrightness);

// Catch until button released for software debounce
    while ( digitalRead(buttonPin) == HIGH ) { BPcycleCount++; delay (100); }
    Serial.print("BPcycleCount: "); Serial.print(BPcycleCount);
// If button is held for 8 seconds the startup sequence is ran.
    if (BPcycleCount>80) { StartupSequence(); }
    BPcycleCount = 1;
  }

// Run knight rider scroll
  KNscroll(KNcolorToDisplay, KNtrailcolorToDisplay);  

// Turn off Mode LEDs
  TurnOffLEDs(1, 8);
  
// Redraw User mode
  DrawUserMode(UserMode);

// Redraw Adaptive Mode
  if (AdaptiveMode>UserMode) {
    DrawAdaptiveMode(UserMode, AdaptiveMode);
  }

}

void SerialDebug() {

  // Serial output for debugging
    Serial.print("Temp: "); Serial.print(temp_c_board); Serial.print("*C\t"); Serial.print(temp_f_board); Serial.println("*F");
    Serial.print("Bank fan speed: "); Serial.println(bankFanSpeed);
    Serial.print("PSU Fan Setting: "); Serial.println(psuFanSpeed);
    Serial.print("User Mode: "); Serial.println(UserMode);
    Serial.print("Adaptive Mode: "); Serial.println(AdaptiveMode);
    Serial.print("Count: "); Serial.println(cycleCount);
    Serial.print("High Temp Count: "); Serial.println(HighTempTicks);
    Serial.print("StatusColor: "); Serial.println(StatusColor);
    Serial.print("MinutesForHighTempReset: "); Serial.println(MinutesForHighTempReset);
    Serial.print("HighTempThresholdInMinutes: "); Serial.println(HighTempThresholdInMinutes);
    Serial.print("MinutesForFanIncrease: "); Serial.println(MinutesForFanIncrease);
    Serial.print("FanSpeedIncrement: "); Serial.println(FanSpeedIncrement);
    Serial.print("MaxTempThreshold: "); Serial.println(MaxTempThreshold);
    Serial.print("OptimalTempThreshold: "); Serial.println(OptimalTempThreshold);
    Serial.print("milliSecondsToSpinForPurge: "); Serial.println(milliSecondsToSpinForPurge);
    Serial.println();
    Serial.println();
    
}

void CheckIfFanSpeedChangesNeeded() {

// Set KN scroll color to WHITE
StatusColor = 1;

// Dials fan speeds up incrementally if above MaxTempThreshold
  if (temp_f_board > MaxTempThreshold ) {
    HighTempTicks++;  
    
    //Serial.println("Temp above MaxTempThreshold.");
    if ( cycleCount > ( TicksPerMinute * MinutesForFanIncrease )) {
        cycleCount = 0;
        //Serial.println("Increasing speed.");
        if (psuFanSpeed <= ( 255 - FanSpeedIncrement)) {
          psuFanSpeed = (psuFanSpeed + FanSpeedIncrement );
          analogWrite(psu_fan, psuFanSpeed);
        }

        if (bankFanSpeed <= ( 255 - FanSpeedIncrement )) {
          bankFanSpeed = (bankFanSpeed + FanSpeedIncrement );
          analogWrite(fan_banks, bankFanSpeed);
        }
       
        // Set KN scroll color to RED
        StatusColor = 2;
    }
// Dials fan speeds down incrementally to the MIN if below MaxTempThreshold
  } else if (bankFanSpeed > bankFanMINSpeed || psuFanSpeed > psuFanMINSpeed ) {
      HighTempTicks++;

      // Reduce fan speed if cycle counter met.
      if ( cycleCount > ( TicksPerMinute * MinutesForFanDecrease)) {
        cycleCount = 0;
        if ( psuFanSpeed >= ( psuFanMINSpeed + FanSpeedIncrement)) {
        psuFanSpeed = (psuFanSpeed - FanSpeedIncrement );
        analogWrite(psu_fan, psuFanSpeed);
        }

        if ( bankFanSpeed >= (bankFanMINSpeed + FanSpeedIncrement)) {
        bankFanSpeed = (bankFanSpeed - FanSpeedIncrement );
        analogWrite(fan_banks, bankFanSpeed);
        }

        // Set KN scroll color to BLUE
        StatusColor = 0;
      }
      
  } else {
    if (cycleCount > (TicksPerMinute * MinutesForHighTempReset)) {
      HighTempTicks = 0;
      cycleCount = (TicksPerMinute * MinutesForFanIncrease); // Need to reset this so it does not buffer reset.
    }
  }
}


void MAXFanSpinToPurgeHotAir() {
  
  HighTempTicks = 0; // Reset 

  // Turn off Mode LEDs
  TurnOffLEDs(1, 16);

  strip.setPixelColor(8, color_red); // Draw new pixel
  strip.setPixelColor(9, color_white); // Draw new pixel
  strip.setPixelColor(10, color_blue); // Draw new pixel
  strip.show();
  
  // Max fans out
  analogWrite(psu_fan, 255);
  analogWrite(fan_banks, 255);

  delay(milliSecondsToSpinForPurge);

// Turn off all LEDs
  TurnOffLEDs(1, 16);
  
// Redraw User mode
  DrawUserMode(UserMode);

// Redraw Adaptive Mode
  if (AdaptiveMode>UserMode) {
    DrawAdaptiveMode(UserMode, AdaptiveMode);
  }

  analogWrite(psu_fan, psuFanSpeed);
  analogWrite(fan_banks, bankFanSpeed);

}

void StartupSequence() {
  
  LEDtest();

  DrawUserMode(UserMode);

  // Fade LED ring on
  for ( int i = 0; i < 255; i++ ) {
        analogWrite(ringPin, i);
        delayMicroseconds(50);
  }

  // Fade LED ring off
  for ( int ii = 255; ii < 0; ii++ ) {
        analogWrite(ringPin, ii);
        delayMicroseconds(50);
  }

  analogWrite(ringPin, ringBrightness);

  // Write the changes to the PWM pins
  analogWrite(psu_fan, 255);
  analogWrite(fan_banks, 255);

  // Delay to allow fans to spin up
  delay(3000);
  
  // Spin fans down to minimum
  psuFanSpeed = 35;
  bankFanSpeed = 25;
  psuFanMINSpeed = psuFanSpeed;
  bankFanMINSpeed = bankFanSpeed;

  // Write the changes to the PWM pins
  analogWrite(psu_fan, psuFanSpeed);
  analogWrite(fan_banks, bankFanSpeed);
  
}



void LEDtest() {

for (int z=15; z>-1; z--) {
  //RED
  for (uint32_t i=0; i<255;) {
    uint32_t c = strip.Color(i, 0, 0);
    writeLED(z, c);
    i=i-5;
  }

  for (uint32_t i=255; i>0;) {
    uint32_t c = strip.Color(i, 0, 0);
    writeLED(z, c);
    i=i-5;
  }
  // BLUE
  for (uint32_t i=0; i<255;) {
    uint32_t c = strip.Color(0, i, 0);
    writeLED(z, c);
    i=i-5;
  }

  for (uint32_t i=255; i>0;) {
    uint32_t c = strip.Color(0, i, 0);
    writeLED(z, c);
    i=i-5;
  }

  // GREEN
  for (uint32_t i=0; i<255;) {
    uint32_t c = strip.Color(0, 0, i);
    writeLED(z, c);
    i=i-5;
  }

  for (uint32_t i=255; i>0;) {
    uint32_t c = strip.Color(0, 0, i);
    writeLED(z, c);
    i=i-5;
  }

  strip.setPixelColor(z, 0); 
  strip.show();
  
}

}

// This function is used only for LEDtest()
void writeLED(int z, uint32_t color) {
    strip.setPixelColor(z, color);  
    strip.show();
    delay(3);
}



static void KNscroll(uint32_t LeadColor, uint32_t TrailColor) {

  // This draws a bright pixel from the first pixel as defined by KnightRiderPixelBegin.
  // This pixel appears to scrolls incrementally down to the last pixel as define by 
  // KnightRiderPixelEnd. This bright LED is followed by a dimmer illumated LED. This
  // effect is similar to the the "Anamorphic Equalizer" on Knight Rider's KITT.

int count=0;
int countdown=KnightRiderPixelEnd-KnightRiderPixelBegin;

  // KN to last pixel
  for(int i=KnightRiderPixelBegin; i<KnightRiderPixelEnd+2; i++) {
      
      if (countdown > -1) { strip.setPixelColor(i, LeadColor); } // Draw new pixel
      if (count > 0 && countdown > -1) { strip.setPixelColor(i-1, TrailColor);  }// To avoid drawing behind the inteded
      if (count > 1) { strip.setPixelColor(i-2, 0); }// Erase pixel a few steps back
      strip.show();
      delay(128);

      count++;
      countdown--;
  }

count=0;
int countup=KnightRiderPixelEnd-KnightRiderPixelBegin; // 13 - 10

  // KN to first pixel
  for(int z=KnightRiderPixelEnd; z>KnightRiderPixelBegin-2; z--) {
      
      if (countup > -1) { strip.setPixelColor(z, LeadColor); } 
      if (count > 0 && countup > -1) { strip.setPixelColor(z+1, TrailColor);  }// To avoid drawing behind the inteded
      if (count > 1) { strip.setPixelColor(z+2, 0); }// Erase pixel a few steps back
      strip.show();
      delay(128);

      count++;
      countup--;
  }
}




void TurnOffLEDs(uint32_t FirstLED, uint32_t LastLED) {
    // Turn off Mode LEDs
  for(int i=FirstLED-1; i<LastLED ; i++) {
      strip.setPixelColor(i , 0); // Draw new pixel
      strip.show();
  }
}




void DrawUserMode(uint32_t LastLED) {
    for(int i=0; i<LastLED ; i++) {
      strip.setPixelColor(i, UMcolorToDisplay); // Draw new pixel
      strip.show();
    }
}





void DrawAdaptiveMode(uint32_t FirstLED, uint32_t LastLED) {
    for(int i=FirstLED; i<LastLED ; i++) {
      strip.setPixelColor(i , AMcolorToDisplay); // Draw new pixel
      strip.show();
    }
}
