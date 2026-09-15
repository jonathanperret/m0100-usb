/* ================================================================================
   Author  : GuilleAcoustic
   Date    : 2015-05-22
   Revision: V1.0
   Purpose : Opto-mechanical trackball firmware
   --------------------------------------------------------------------------------
   Modified for use with Apple M0100 mouse
   By Johan Berglund, 2015-08-10

   Changes in code:
   - Internal pullup set for pin 14 (B3)
   - State check for right and middle buttons commented out

   Ported to RP2040 Zero
   By Jonathan Perret (https://github.com/jonathanperret), 2025-11-25

   Changes in code:
   - update pin numbers
   - use `digitalRead` instead of port manipulation
   - add MOUSE_SPEED constant to speed up movement

   Connection to DB9 via adapter board:
   
   DB9     M0100 RP2040 Zero
    1      GND   GND
    2      5V    5V
    3      GND   GND
    4      X2    12
    5      X1    13
    6      -     -       (not connected)
    7      SW    14
    8      Y2    15
    9      Y1    26
   
   ================================================================================ */
#include <Mouse.h>

const int PIN_SWITCH = 14;
const int PIN_X1 = 13;
const int PIN_X2 = 12;
const int PIN_Y1 = 26;
const int PIN_Y2 = 15;

const int MOUSE_SPEED = 2;

// =================================================================================
// Type definition
// =================================================================================
typedef struct
{
  int8_t  coordinate = 0;
  uint8_t index      = 0;
} ENCODER_;

// =================================================================================
// Constant definition
// =================================================================================
const int8_t lookupTable[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1,  1,  0};

// =================================================================================
// Volatile variables
// =================================================================================
volatile ENCODER_ xAxis;
volatile ENCODER_ yAxis;

// =================================================================================
// Setup function
// =================================================================================
void setup()
{

  // Set pull-up for mouse switch on M0100
  pinMode(PIN_SWITCH, INPUT_PULLUP);

  // The encoder inputs must be configured explicitly: on the RP2040, GPIO 26-29
  // (the ADC-capable pins) start with their digital input buffer disabled, so
  // without a pinMode() call digitalRead(26) always returns LOW and the Y axis
  // only wiggles. pinMode() enables the input buffer.
  pinMode(PIN_X1, INPUT_PULLUP);
  pinMode(PIN_X2, INPUT_PULLUP);
  pinMode(PIN_Y1, INPUT_PULLUP);
  pinMode(PIN_Y2, INPUT_PULLUP);
 
  // Attach interruption to encoders channels
  attachInterrupt(digitalPinToInterrupt(PIN_X1), ISR_HANDLER_X, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_X2), ISR_HANDLER_X, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_Y1), ISR_HANDLER_Y, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_Y2), ISR_HANDLER_Y, CHANGE);
 
  // Start the mouse function
  Mouse.begin();
}

// =================================================================================
// Main program loop
// =================================================================================
void loop()
{
  // Update mouse coordinates
  if (xAxis.coordinate != 0 || yAxis.coordinate != 0)
  {
    Mouse.move(MOUSE_SPEED * xAxis.coordinate, MOUSE_SPEED * yAxis.coordinate);
    xAxis.coordinate = 0;
    yAxis.coordinate = 0;
  }

  // Update buttons state
  if (digitalRead(PIN_SWITCH) == LOW) Mouse.press(MOUSE_LEFT); else Mouse.release(MOUSE_LEFT);

  // Wait a little before next update
  delay(10);
}

// =================================================================================
// Interrupt handlers
// =================================================================================
void ISR_HANDLER_X()
{
  // Build the LUT index from previous and new data
  xAxis.index       = (xAxis.index << 2) | (digitalRead(PIN_X1) << 1) | digitalRead(PIN_X2);
  xAxis.coordinate += lookupTable[xAxis.index & 0b1111];
}

void ISR_HANDLER_Y()
{
  // Build the LUT index from previous and new data
  yAxis.index       = (yAxis.index << 2) | (digitalRead(PIN_Y1) << 1) | digitalRead(PIN_Y2);
  yAxis.coordinate += lookupTable[yAxis.index & 0b1111];
}
