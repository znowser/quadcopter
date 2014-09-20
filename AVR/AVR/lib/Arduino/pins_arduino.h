/*Modified version that supports the ATMEGA128*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

//Not used by this implementation, undefine if defined
#ifdef analogPinToChannel
	#undef analogPinToChannel
#endif

#define ADC0 0
#define ADC1 1
#define ADC2 2
#define ADC3 3
#define ADC4 4

#ifndef PA
	#define PA 1
	#define PB 2
	#define PC 3
	#define PD 4
	#define PE 5
	#define PF 6
	#define PG 7
#endif


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)

//__AVR_ATmega128__
extern const uint16_t PROGMEM port_to_mode_PGM[];
extern const uint16_t PROGMEM port_to_output_PGM[];
extern const uint16_t PROGMEM port_to_input_PGM[];
extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];


extern const uint8_t SS;
extern const uint8_t MOSI;
extern const uint8_t MISO;
extern const uint8_t SCK;

extern const uint8_t SDA;
extern const uint8_t SCL;



// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR			Serial
#define SERIAL_PORT_HARDWARE		Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif