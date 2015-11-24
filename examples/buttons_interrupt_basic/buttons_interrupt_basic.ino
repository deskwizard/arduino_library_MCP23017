#define DEBUG_LED 13            // Debugging LED on pin 13

#include <MCP23017.h>           // Include MCP23017 library

/*
  See https://www.arduino.cc/en/Reference/AttachInterrupt for interrupt # to pin mapping.
  All the pins listed in the table on that page should work for:
  All 168/328p based (e.g Uno, Diecimila, Duemilanove), Ethernet, Mega2560 and 32u4 based (e.g Leonardo, Micro).

  Note:
  Arduino Due can use pins: 0 through 5, Arduino Zero can use 0,1,2,3,5.
  In both cases (except pin 4 for the Arduino Zero) other pins requires minor additions to the library.
  Have a look at the library, it's pretty easy to add any other pins you might want to use.

  Might work on Attiny45/85 (Untested).
*/

// Setup expander to use interrupt pin 0 (Digital pin 2 on 168/328p).
MCP23017 expander(0);


void setup() {

  expander.begin();             // Start expander

  Serial.begin(115200);
  Serial.println("MCP23017 Button handling demo sketch starting...");
  Serial.println();
  pinMode(DEBUG_LED, OUTPUT);

} // **** End setup() ****

void loop() {

  expander.handleClicks();

} // **** End loop() ****
