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
    8      Y2    26
    9      Y1    15
   
   ================================================================================ */
// The Adafruit TinyUSB stack is used instead of the core's default Pico SDK one,
// selected with -DUSE_TINYUSB (see flash.command). The Pico SDK stack gives the
// sketch no control over the HID interface descriptor: it hardcodes
// HID_ITF_PROTOCOL_NONE and puts a report ID in the mouse report, neither of
// which an ordinary mouse does.
//
// Apple's Accessory Design Guidelines ch. 55 specify no report descriptor
// requirements for mice at all (unlike keyboards and trackpads, which are set
// out in detail), and do not mention boot protocol. In IOHIDFamily boot
// protocol is only a fallback - parseRelativeElement() drops the boot mouse
// path as soon as it finds a relative Generic Desktop X/Y element. So declaring
// boot protocol here is not required by anything; it is kept only because it is
// what a real mouse does and it costs nothing.
#include <Adafruit_TinyUSB.h>

// =================================================================================
// USB identity
// =================================================================================
const char     USB_MANUFACTURER_NAME[] = "m0100-usb";
const char     USB_PRODUCT_NAME[]      = "M0100 USB adapter";
const uint16_t USB_VENDOR_ID           = 0x2E8A;   // Raspberry Pi
const uint16_t USB_PRODUCT_ID          = 0x0003;
const uint16_t USB_MAX_POWER_MA        = 100;      // see README, empirical

// Expose the HID mouse on its own, with no USB serial port beside it, so the
// adapter is a plain single-interface HID device like an ordinary USB mouse
// rather than a composite one (bDeviceClass 0 instead of 0xEF / IAD).
//
// The cost is the 1200 baud reboot-into-bootloader trick: without the serial
// port, flash.command cannot reboot the board itself, so it has to be plugged
// in with BOOT held before flashing. Set this to 0 to get the serial port and
// one-step flashing back.
#define USB_HID_ONLY 1

// The M0100 is a one button mouse with no wheel, so the report descriptor is
// written out here rather than using TinyUSB's TUD_HID_REPORT_DESC_MOUSE(),
// which has a fixed 5 button + wheel + pan layout and would describe hardware
// that does not exist.
//
// The result is one button bit, seven bits of padding, and relative X and Y:
// three bytes, which is exactly the HID boot mouse report. Boot protocol and
// report protocol are therefore identical here, and the descriptor carries no
// report ID (boot protocol does not allow one).
const uint8_t desc_hid_report[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x09, 0x01,        //   Usage (Pointer)
  0xA1, 0x00,        //   Collection (Physical)
  0x05, 0x09,        //     Usage Page (Button)
  0x19, 0x01,        //     Usage Minimum (Button 1)
  0x29, 0x01,        //     Usage Maximum (Button 1)
  0x15, 0x00,        //     Logical Minimum (0)
  0x25, 0x01,        //     Logical Maximum (1)
  0x95, 0x01,        //     Report Count (1)
  0x75, 0x01,        //     Report Size (1)
  0x81, 0x02,        //     Input (Data, Variable, Absolute)
  0x95, 0x01,        //     Report Count (1)
  0x75, 0x07,        //     Report Size (7)
  0x81, 0x03,        //     Input (Constant) - padding to a whole byte
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x30,        //     Usage (X)
  0x09, 0x31,        //     Usage (Y)
  0x15, 0x81,        //     Logical Minimum (-127)
  0x25, 0x7F,        //     Logical Maximum (127)
  0x75, 0x08,        //     Report Size (8)
  0x95, 0x02,        //     Report Count (2)
  0x81, 0x06,        //     Input (Data, Variable, Relative)
  0xC0,              //   End Collection
  0xC0               // End Collection
};

// The three bytes of that report, in order.
typedef struct __attribute__((packed))
{
  uint8_t buttons;
  int8_t  x;
  int8_t  y;
} MOUSE_REPORT_;

// 8 ms, i.e. 125 Hz, the usual rate for an ordinary USB mouse. (1 ms is gaming
// mouse territory and would reserve bus bandwidth every frame for no benefit:
// the M0100 is a ~90 CPI opto-mechanical mouse, so even a fast swipe produces
// only a few dozen counts per 8 ms.)
//
// The rate is not what made movement smooth - see the loop, which reports when
// the endpoint is free rather than on a timer of its own.
const uint8_t HID_POLL_INTERVAL_MS = 8;

Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report),
                          HID_ITF_PROTOCOL_MOUSE, HID_POLL_INTERVAL_MS, false);

const int PIN_SWITCH = 14;
const int PIN_X1 = 13;
const int PIN_X2 = 12;
const int PIN_Y1 = 15;
const int PIN_Y2 = 26;

// Pointer speed, as 8.8 fixed point. The M0100 is a low resolution mouse (about
// 90 counts per inch against 400+ for a modern one) so its counts need scaling
// up. Doing that with a plain integer multiply makes every report jump by a
// whole multiple of it, which is visibly steppy; the fixed point remainder
// carried in xSubPixel/ySubPixel below spreads the fraction over reports
// instead, and leaves iOS a smooth stream to run its own acceleration on.
const int32_t MOUSE_SPEED_Q8 = 2 * 256;   // 2.0

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
 
  // Bring the USB device up as a boot protocol mouse. main() has already called
  // TinyUSBDevice.begin(), which builds the descriptor from scratch and attaches,
  // so the identity and the HID interface have to be applied here and the device
  // re-attached for the host to read the new descriptor.
  TinyUSBDevice.detach();
#if USB_HID_ONLY
  // Drops the CDC serial interface begin() added, and resets bDeviceClass back
  // to 0. It also clears the identity, so that is set again below.
  TinyUSBDevice.clearConfiguration();
#endif
  TinyUSBDevice.setID(USB_VENDOR_ID, USB_PRODUCT_ID);
  TinyUSBDevice.setManufacturerDescriptor(USB_MANUFACTURER_NAME);
  TinyUSBDevice.setProductDescriptor(USB_PRODUCT_NAME);
  TinyUSBDevice.setConfigurationMaxPower(USB_MAX_POWER_MA);
  usb_hid.begin();
  delay(10);
  TinyUSBDevice.attach();
}

// A HID mouse report carries one signed byte per axis. Anything beyond that is
// kept in the accumulator for the next report rather than being clipped away.
int8_t takeDelta(int32_t &value)
{
  int8_t out;
  if (value > 127) {
    out = 127;
  } else if (value < -127) {
    out = -127;
  } else {
    out = (int8_t)value;
  }
  value -= out;
  return out;
}

// =================================================================================
// Main program loop
// =================================================================================
void loop()
{
  // Remainders of the fixed point scaling, carried over to the next report.
  static int32_t xSubPixel = 0;
  static int32_t ySubPixel = 0;
  static int32_t xPending = 0;
  static int32_t yPending = 0;
  static uint8_t previousButtons = 0;

  // The host's polling is the only clock here. usb_hid.ready() is false until
  // the host has collected the previous report, so looping freely and sending
  // whenever it goes true delivers exactly one report per poll, each carrying
  // the counts accumulated since the last one.
  //
  // An earlier version also rate-limited on millis(), which was the cause of
  // the jagged movement rather than a cure for it: a software timer and the USB
  // frame clock are independent, so they drift against each other and some
  // polls ended up carrying two batches of counts while others carried none.
  if (!TinyUSBDevice.mounted() || !usb_hid.ready()) {
    return;
  }

  // Take the counts the interrupt handlers have accumulated since last time.
  noInterrupts();
  int32_t xCount = xAxis.coordinate;
  int32_t yCount = yAxis.coordinate;
  xAxis.coordinate = 0;
  yAxis.coordinate = 0;
  interrupts();

  // Scale, keeping the fractional part for the next report.
  xSubPixel += xCount * MOUSE_SPEED_Q8;
  ySubPixel += yCount * MOUSE_SPEED_Q8;
  xPending += xSubPixel >> 8;
  yPending += ySubPixel >> 8;
  xSubPixel &= 0xff;
  ySubPixel &= 0xff;

  uint8_t buttons = (digitalRead(PIN_SWITCH) == LOW) ? 0x01 : 0x00;

  // Nothing to say: stay quiet rather than flood the host with empty reports.
  if (xPending == 0 && yPending == 0 && buttons == previousButtons) {
    return;
  }

  // sendReport rather than mouseReport: the latter always sends TinyUSB's
  // 5 byte buttons/x/y/wheel/pan layout, which no longer matches the descriptor.
  MOUSE_REPORT_ report;
  report.buttons = buttons;
  report.x = takeDelta(xPending);
  report.y = takeDelta(yPending);
  usb_hid.sendReport(0, &report, sizeof(report));
  previousButtons = buttons;
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
