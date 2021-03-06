#ifdef __AVR_ATtiny85__
#include <TinyWireM.h>
#define Wire TinyWireM
#else
#include <Wire.h>
#endif


#ifdef __AVR
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif
#include "MCP23017.h"
#include <Arduino.h>


// **** GPIO related class instances ****

void MCP23017::setGPIO(uint8_t pin, uint8_t value){
//	// read register ** Change later **
//    readGPIO = readReg(MCP_GPIOB);

	Serial.print(F("GPIOB register read before: "));
	Serial.println(readGPIO);

	// Change required pin here.
	Serial.print(F("Pin "));
	Serial.print(pin);
	Serial.print(F(" value requested: "));
	Serial.println(value);
	Serial.print(F("Current pin "));
	Serial.print(pin);	
	Serial.print(F(" value: "));
	Serial.println(bitRead(readGPIO, pin));

	// Change bit
	switch (value) {
	case LOW:
		Serial.println(F(" - LOW - "));
		bitClear(readGPIO, pin);
	break;
	case HIGH:
		Serial.println(F(" - HIGH - "));
		bitSet(readGPIO, pin);
	break;
	case INV:  // *** Bug: Will only work once (3 presses, wont turn back off after that)
		Serial.println(F(" - INV - "));
		bitToggle(readGPIO, pin);
	break;
	}

	Serial.print(F("Pin "));
	Serial.print(pin);
	Serial.print(F(" value set to: "));
	Serial.println(bitRead(readGPIO, pin));
	Serial.print(F("Current pin value: "));
	Serial.println(bitRead(readGPIO, pin));

	// Update MCP
	writeReg(MCP_GPIOB, readGPIO);

	readGPIO = readReg(MCP_GPIOB);
	Serial.print(F("GPIOB register read after: "));
	Serial.println(bitRead(readGPIO, pin));
	Serial.println();
}


// **** Buttons related class instances ****

uint8_t MCP23017::buttonRead(uint8_t _buttonID) {
  uint8_t retval = 0x00;
#ifdef _MCP_SERIAL_DEBUG2
  Serial.print(F("Handling (read) for buttonID: "));
  Serial.print(_buttonID);
  Serial.print(" : ");
  Serial.println(buttonNeedHandling[_buttonID]);
#endif
  retval = buttonClickType[_buttonID];
  if (buttonNeedHandling[_buttonID] == true) {
#ifdef _MCP_SERIAL_DEBUG4
    Serial.print(F("Button handling (read) Required for Button ID: "));
    Serial.println(_buttonID);
#endif
    buttonNeedHandling[_buttonID] = false;
    retval |= 0x80;
  }

  return retval;
}


// Class instance to get what pin caused the interrupt
uint8_t MCP23017::getLastIntPin() {

  // We need to get the interrupt pin before disabling interrupts
  // because INTFTFx gets cleared when we disable interrupts.
  uint8_t intpin = readReg(MCP_INFTFA);

  writeReg(MCP_GPINTENA, 0x00);     // Disable interrupts on MCP23017

  for (uint8_t x = 0; x < last_button; x++) {   // Check which GPIO Port A pin caused the interrupt
    if (bitRead(intpin, x)) {
      return x;           // Return the pin number (Port A bit)
    }
  }
  return MCP_ERR;             // Return error if no pin cause the interrupt (since it makes no sense...)
}

void MCP23017::handleClicks() {

  uint32_t currentMillis = millis();    // Save current millis()

  for (uint8_t x = 0; x < last_button; x++) {  // loop through buttons

    // Single/Double click handling
    if (currentMillis - buttonReleaseTime[x] >= 225) {
      if (buttonReleaseCount[x] >= 2) {
        buttonNeedHandling[x] = true;
        buttonClickType[x] = MCP_DOUBLE_CLICK;
        buttonReleaseCount[x] = 0;
#ifdef _MCP_SERIAL_DEBUG
        Serial.print(F("Double click on button "));
        Serial.println(x);
        Serial.println();
#endif
      }
      else if (buttonReleaseCount[x] == 1) {
        buttonNeedHandling[x] = true;
        buttonClickType[x] = MCP_CLICK;
        buttonReleaseCount[x] = 0;
#ifdef _MCP_SERIAL_DEBUG
        Serial.print(F("Single click on button "));
        Serial.println(x);
        Serial.println();

      }

      if (buttonNeedHandling[x] == true) {
#ifdef _MCP_SERIAL_DEBUG3
        Serial.print(F("Button handling (handle) Required for Button ID: "));
        Serial.println(x);
#endif
#endif
      }

    } // end single/double click handling

    // Button hold handling
    if (currentMillis - buttonPressTime[x] > MCP_LONG_CLICK_TIMEOUT && buttonState[x] && buttonClickType[x] != MCP_HELD_DOWN) {
      buttonNeedHandling[x] = true;
      buttonClickType[x] = MCP_HELD_DOWN;
#ifdef _MCP_SERIAL_DEBUG
      Serial.print(F("Button "));
      Serial.print(x);
      Serial.println(F(" is held down"));
      Serial.println();
#endif
    }
  } // end button loop

  if (ISR_flag) {      // If MCP23017 Interrupt flag is set....

    uint8_t _buttonID = getLastIntPin();  // Get which pin caused the interrupt
    uint32_t isr_time_diff = (uint32_t)(currentMillis - prevISRMillis); // Time difference between the time the ISR fired and now

    if (isr_time_diff >= MCP_ISR_DEBOUNCE) {  // If we have a difference of MCP_ISR_DEBOUCE...

      uint8_t _buttonState = readReg(MCP_INTCAPA);          // Read port state at the time of interrupt (INTCAP register)

      ISR_flag = false;         // Reset interrupt flag

      if (_buttonID != MCP_ERR) {   // If the function didn't return ERR...
        if (_buttonState != MCP_ERR) {

          buttonState[_buttonID] = bitRead(_buttonState, _buttonID);  // Set the button state accordingly

          if (buttonState[_buttonID]) { // Pressed
            buttonNeedHandling[_buttonID] = true;
            buttonClickType[_buttonID] = MCP_PRESS;
#ifdef _MCP_SERIAL_DEBUG
            Serial.print(F("Button press on button: "));
            Serial.println(_buttonID);
            Serial.println();
#endif
#ifdef _MCP_SERIAL_DEBUG2
            Serial.print(F("Press Time for button:  "));
            Serial.print(_buttonID);
            Serial.print(F(" : "));
            Serial.println(currentMillis - buttonPressTime[_buttonID]);
            Serial.println();
#endif
            buttonPressTime[_buttonID] = currentMillis;  // Save the time at which the button was pressed (approximately...)

          }
          else {                  // Released
#ifdef _MCP_SERIAL_DEBUG2
            Serial.print(F("Release count: "));
            Serial.print(buttonReleaseCount[_buttonID]);
            Serial.print(F("  Time: "));
            Serial.println(currentMillis - buttonReleaseTime[_buttonID]);
            Serial.println();
#endif
            if (isr_time_diff < MCP_LONG_CLICK_LENGTH) {
              buttonReleaseCount[_buttonID]++;
            }
            else if (isr_time_diff > MCP_LONG_CLICK_LENGTH && (currentMillis - buttonPressTime[_buttonID]) <= MCP_LONG_CLICK_TIMEOUT) {
              buttonNeedHandling[_buttonID] = true;
              buttonClickType[_buttonID] = MCP_LONG_CLICK;
#ifdef _MCP_SERIAL_DEBUG
              Serial.print(F("Long click on button "));
              Serial.println(_buttonID);
              Serial.println();
#endif
            }
            if (currentMillis - buttonPressTime[_buttonID] > MCP_LONG_CLICK_TIMEOUT) {

              if (buttonClickType[_buttonID] == MCP_HELD_DOWN) {
                buttonNeedHandling[_buttonID] = true;
                buttonClickType[_buttonID] = MCP_HELD_RELEASE;
#ifdef _MCP_SERIAL_DEBUG
                Serial.print(F("Button "));
                Serial.print(_buttonID);
                Serial.println(F(" was held down, now is released"));
                Serial.println();
#endif
              }
            }
            buttonPressTime[_buttonID] = 0;
            buttonReleaseTime[_buttonID] = currentMillis;
          }
        }
      } // end if _buttonID

      prevISRMillis = currentMillis;  // Save when we last checked for interrupt (approximately...)
      attach_isr();         // Re-enable interrupts on Arduino
    } // end debounce
  }
}


// Class instance to configure the MCP23017
void MCP23017::configure()
{

// expander configuration register
  writeReg(MCP_IOCONA, 0x60);         // Disable sequential mode (0b01100000)
  writeReg(MCP_IOCONB, 0x60);         // Disable sequential mode (0b01100000)

  uint8_t tempGPPUA = 0;
  uint8_t tempIOPOLA = 0;
  uint8_t tempGPINTENA = 0;
  uint8_t tempIODIRA = 0;
  uint8_t tempGPIOA = 0;

  uint8_t tempGPPUB = 0;
  uint8_t tempIOPOLB = 0;
  uint8_t tempGPINTENB = 0;
  uint8_t tempIODIRB = 0;
  uint8_t tempGPIOB = 0;

  // Configure Buttons...
  for (uint8_t pin = 0; pin < last_button; pin++) {
//	Serial.print(F("Configuring buttons..."));
//	Serial.println(pin);

	bitSet(tempGPPUA, pin);
	bitSet(tempIOPOLA, pin);
	bitSet(tempGPINTENA, pin);
	bitSet(tempIODIRA, pin);
  	if (last_button > 8) {
//	  Serial.println(F("PORTB"));
	  bitSet(tempGPPUB, pin);
      bitSet(tempIOPOLB, pin);
	  bitSet(tempGPINTENB, pin);
	  bitSet(tempIODIRB, pin);	
	}
  }

    // Configure outputs...
  for (uint8_t pin = last_button; pin < (last_button + last_output); pin++) {
//	Serial.print(F("Configuring outputs..."));
//	Serial.println(pin);

  	if (last_button > 8) {
//	  Serial.println(F("PORTB"));

	}
  }
/*
  Serial.print(F("tempGPPUA: "));
  Serial.println(tempGPPUA);
  Serial.print(F("tempIOPOLA: "));
  Serial.println(tempIOPOLA);
  Serial.print(F("tempGPINTENA: "));
  Serial.println(tempGPINTENA);
  Serial.print(F("tempIODIRA: "));
  Serial.println(tempIODIRA);
  Serial.print(F("tempGPIOA: "));
  Serial.println(tempGPIOA);
  Serial.println();
  Serial.print(F("tempGPPUB: "));
  Serial.println(tempGPPUB);
  Serial.print(F("tempIOPOLB: "));
  Serial.println(tempIOPOLB);
  Serial.print(F("tempGPINTENB: "));
  Serial.println(tempGPINTENB);
  Serial.print(F("tempIODIRB: "));
  Serial.println(tempIODIRB);
  Serial.print(F("tempGPIOB: "));
  Serial.println(tempGPIOB);
*/
  // Write Port A configuration
  writeReg(MCP_GPPUA, tempGPPUA);            // Enable pull-up resistors - Port A
  writeReg(MCP_IOPOLA, tempIOPOLA);          // Invert polarity of signal - Port A
  writeReg(MCP_GPINTENA, tempGPINTENA);      // Enable interrupts - Port A
  writeReg(MCP_IODIRA, tempIODIRA);          // Set port A direction
  writeReg(MCP_GPIOA, tempGPIOA);            // Set port A GPIO values

  // Write Port A configuration
  writeReg(MCP_GPPUB, tempGPPUB);            // Enable pull-up resistors - Port B
  writeReg(MCP_IOPOLB, tempIOPOLB);          // Invert polarity of signal - Port B
  writeReg(MCP_GPINTENB, tempGPINTENB);      // Enable interrupts - Port B
  writeReg(MCP_IODIRB, tempIODIRB);          // Set port B direction
  writeReg(MCP_GPIOB, tempGPIOB);            // Set port B GPIO values

  readGPIO = readReg(MCP_GPIOB);		// Load pin states

  readReg(MCP_INTCAPA);                 // Read from interrupt capture port A to clear it
  readReg(MCP_INTCAPB);                 // Read from interrupt capture port B to clear it
}


// **** Read/Write routines ****

// Class instance to write a register to MCP23017
void MCP23017::writeReg(const uint8_t reg, const uint8_t data)
{
  Wire.beginTransmission(MCP_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// Class instance to write both Port A and B registers to MCP23017
void MCP23017::writeBothReg(const uint8_t reg, const uint8_t data)
{
  Wire.beginTransmission(MCP_ADDRESS);
  Wire.write(reg);
  Wire.write(data);  // port A
  Wire.write(data);  // port B
  Wire.endTransmission();
}

// Class instance to read a register from MCP23017
uint8_t MCP23017::readReg(const uint8_t reg)
{
  Wire.beginTransmission(MCP_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MCP_ADDRESS, (uint8_t)1);
  return Wire.read();
}

// Class instance to configure the MCP23017 and prepare arduino interrupts
void MCP23017::begin()
{
  configure();    // Configure MCP23017
  attach_isr();   // Configure Interrupt on Arduino
}

void MCP23017::detach_isr() {
  // AttachInterrupt according to the value passed when class is created in sketch

  switch (whichISR_)
  {
    case 0:
      detachInterrupt(0);
      instance0_ = this;
      break;
    case 1:
      detachInterrupt(1);
      instance1_ = this;
      break;
    case 2:
      detachInterrupt(2);
      instance2_ = this;
      break;
    case 3:
      detachInterrupt(3);
      instance3_ = this;
      break;
    case 4:
      detachInterrupt(4);
      instance4_ = this;
      break;
    case 5:
      detachInterrupt(5);
      instance5_ = this;
      break;
  }

  EIFR |= 0x01; // Clear ISR Flag so it doesn't trigger when we reattach
}

void MCP23017::attach_isr() {
  // AttachInterrupt according to the value passed when class is created in sketch
  switch (whichISR_)
  {
    case 0:
      attachInterrupt(0, isr0, FALLING);
      instance0_ = this;
      break;
    case 1:
      attachInterrupt(1, isr1, FALLING);
      instance1_ = this;
      break;
    case 2:
      attachInterrupt(2, isr2, FALLING);
      instance2_ = this;
      break;
    case 3:
      attachInterrupt(3, isr3, FALLING);
      instance3_ = this;
      break;
    case 4:
      attachInterrupt(4, isr4, FALLING);
      instance4_ = this;
      break;
    case 5:
      attachInterrupt(5, isr5, FALLING);
      instance5_ = this;
      break;
  }
    readReg(MCP_INTCAPA);           // Read from interrupt capture port A register (INTCAP register) to clear it
    writeReg(MCP_GPINTENA, 0xFF); // Re-enable interrupts on MCP23017
}

// Class constructor (needs to be placed after "begin()" routine in library)
MCP23017::MCP23017 (const uint8_t address, const uint8_t whichISR, const uint8_t button_cnt, const uint8_t output_cnt) : whichISR_ (whichISR)
{
  // Set address
  MCP_ADDRESS = address;

  // Set last button
  last_button = button_cnt;

  // Set last GPIO
  last_output =  output_cnt;

  Wire.begin(); // Start i2c bus
}

// ISR0 glue class
void MCP23017::isr0()
{
  instance0_->handleInterrupt();
}

// ISR1 glue class
void MCP23017::isr1()
{
  instance1_->handleInterrupt();
}

// ISR2 glue class
void MCP23017::isr2()
{
  instance2_->handleInterrupt();
}

// ISR3 glue class
void MCP23017::isr3()
{
  instance3_->handleInterrupt();
}

// ISR4 glue class
void MCP23017::isr4()
{
  instance4_->handleInterrupt();
}

// ISR5 glue class
void MCP23017::isr5()
{
  instance5_->handleInterrupt();
}

// For use by ISR glue routines
MCP23017 * MCP23017::instance0_;
MCP23017 * MCP23017::instance1_;
MCP23017 * MCP23017::instance2_;
MCP23017 * MCP23017::instance3_;
MCP23017 * MCP23017::instance4_;
MCP23017 * MCP23017::instance5_;

// Class instance to handle an interrupt
void MCP23017::handleInterrupt()
{
  ISR_flag = true;      // Set MCP23017 Interrupt flag
  detach_isr();       // Disable Interrupts on Arduino
}

