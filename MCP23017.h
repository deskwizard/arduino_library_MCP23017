#ifndef _MCP23017_H_
#define _MCP23017_H_

#ifdef __AVR_ATtiny85__
#include <TinyWireM.h>
#else
#include <Wire.h>
#endif

#define _MCP_SERIAL_DEBUG		// Displays clicks, double clicks, long clicks, held down and held release events
#define _MCP_SERIAL_DEBUG		// Displays press / release time
class MCP23017
{

  public:	// Routines accessible by Arduino sketches

    // Starts I2C bus for MCP on address 'address', prepare INT(which) for 'button_count' buttons 
	// and configure 'output_count' outputs
    MCP23017(const uint8_t address, const uint8_t which, const uint8_t button_count, const uint8_t output_count);		
    void begin();						// configures Arduino INT(which) pin and MCP23017 registers

	// Button related
	void handleClicks();				// Button click handler function
	uint8_t buttonRead(uint8_t _buttonID);

	// GPIO related
	void setGPIO(uint8_t pin, uint8_t value);

	// Direct register access
	void writeReg(const uint8_t reg, const uint8_t data);			// Write a register to MCP23017
    void writeBothReg(const uint8_t reg, const uint8_t data);		// Write both registers to MCP23017
	uint8_t readReg(const uint8_t reg);								// Read a register from MCP23017

	private:
	// Routines internal to the library

	void configure();					// configures MCP23017

	// Interrupt related
    const uint8_t whichISR_;			// ISR ID (INT0 or INT1)
    static void isr0 ();				// INT0 ISR
    static void isr1 ();				// INT1 ISR
    static void isr2 ();				// INT2 ISR
    static void isr3 ();				// INT3 ISR
    static void isr4 ();				// INT4 ISR
    static void isr5 ();				// INT5 ISR
    static MCP23017 * instance0_;
    static MCP23017 * instance1_;
    static MCP23017 * instance2_;
    static MCP23017 * instance3_;
    static MCP23017 * instance4_;
    static MCP23017 * instance5_;
	void attach_isr();					// Arduino interrupt attach
	void detach_isr();					// Arduino interrupt detach
	volatile bool ISR_flag = false;		// MCP23017 Interrupt flag
    void handleInterrupt ();			// Interrupt Handler

	// Button related
	uint8_t getLastIntPin();				// Gets which pin caused the interrupt
	uint32_t prevISRMillis = 0; 			// Tracks the time since ISR fired last
	uint32_t buttonPressTime[8] = {0};		// Time the button was pressed
	uint32_t buttonReleaseTime[8] = {0};	// Time the button was released
	uint8_t buttonReleaseCount[8] = {0};	// Button release counter (single/double click handling)
    bool buttonState[8] = {false};			// State of the keys (Up/Down)
	bool buttonNeedHandling[8] = {false};	// Button handling required flag
	uint8_t buttonClickType[8] = {0};		// Button press type (Long, short, double, etc...)	

	uint8_t readGPIO;
};  

	// Button related (Public) 
	inline bool is_new(uint8_t e) {return e & 0x80;}
	inline uint8_t type_of_e(uint8_t e) {return e & 0x7F;}
	inline bool is_new_of_type(uint8_t e, uint8_t t) {return is_new(e) && type_of_e(e) == t;}
	

// Definitions

#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))

#define MCP_ADDRESS 0x20     // MCP23017 is on address 0x20 (all addresses pins low)
#define MCP_ERR 255
#define MCP_ISR_DEBOUNCE 50	 // MCP23017 interrupt debouce time

#define INV 2				 // Used for GPIO (LOW = 0, HIGH = 1, INV = toggle)

// Types of button clicks
#define MCP_PRESS			1	 // ID for button press
#define MCP_CLICK			2	 // ID for clicks (also button release)
#define MCP_DOUBLE_CLICK	3    // ID for double clicks
#define MCP_LONG_CLICK 		4 	 // ID for long clicks
#define MCP_HELD_DOWN 		5 	 // ID for button held down
#define MCP_HELD_RELEASE	6 	 // ID for button held down

// Button timeout value in ms
#define MCP_LONG_CLICK_TIMEOUT (MCP_LONG_CLICK_LENGTH * 3)

// Button long click minimum length in ms (ie. maximum short click length)
#define MCP_LONG_CLICK_LENGTH	500		// Button press is considered a "long press" after this value (in ms)

// MCP23017 registers (everything except direction defaults to 0)
#define MCP_IODIRA   	0x00   // IO direction  (0 = output, 1 = input (Default))
#define MCP_IODIRB   	0x01
#define MCP_IOPOLA  	0x02   // IO polarity   (0 = normal, 1 = inverse)
#define MCP_IOPOLB  	0x03
#define MCP_GPINTENA	0x04   // Interrupt on change (0 = disable, 1 = enable)
#define MCP_GPINTENB 	0x05
#define MCP_DEFVALA  	0x06   // Default comparison for interrupt on change (interrupts on opposite)
#define MCP_DEFVALB  	0x07
#define MCP_INTCONA  	0x08   // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define MCP_INTCONB  	0x09
#define MCP_IOCONA   	0x0A   // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
#define MCP_IOCONB   	0x0B   // same as 0x0A
#define MCP_GPPUA    	0x0C   // Pull-up resistor (0 = disabled, 1 = enabled)
#define MCP_GPPUB    	0x0D
#define MCP_INFTFA   	0x0E   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define MCP_INFTFB   	0x0F
#define MCP_INTCAPA  	0x10   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define MCP_INTCAPB  	0x11
#define MCP_GPIOA    	0x12   // Port value. Write to change, read to obtain value
#define MCP_GPIOB    	0x13
#define MCP_OLLATA   	0x14   // Output latch. Write to latch output. *** Read only reads latch, not port pin! ***
#define	MCP_OLLATB   	0x15

#endif //ifndef _MCP23017_H_
