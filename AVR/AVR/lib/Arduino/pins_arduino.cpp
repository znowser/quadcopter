#include <avr/pgmspace.h>
#include "Arduino.h"
#include "pins_arduino.h"

/*Modified version that supports the ATMEGA128*/

const uint8_t SS   = 10;
const uint8_t MOSI = 12;
const uint8_t MISO = 13;
const uint8_t SCK  = 11;

const uint8_t SDA = 26;
const uint8_t SCL = 25;


// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)

//__AVR_ATmega128__
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
	(uint16_t) &DDRG,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
	(uint16_t) &PORTG,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
	(uint16_t) &PING,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST		
	//Maps pins in the arduino library to exactly the same as the atmega128s layout.
	//-------------------------------------------		
	NOT_A_PIN,	//pin 0
	NOT_A_PIN,	//pin 0
	PE,			//pin ...
	PE,
	PE,
	PE,
	PE,
	PE,
	PE,
	PE,

	PB,
	PB,
	PB,
	PB,
	PB,
	PB,
	PB,
	PB,
	
	PG,
	PG,
	NOT_A_PIN,	//RESET
	NOT_A_PIN,	//VCC
	NOT_A_PIN,	//GND
	NOT_A_PIN,	//XTAL2
	NOT_A_PIN,	//XTAL1

	PD,
	PD,
	PD, 
	PD,
	PD,
	PD, 
	PD,
	PD,
	
	PG,
	PG,

	PC,
	PC,
	PC,
	PC,
	PC,
	PC,
	PC,
	PC,
	
	PG,
	
	PA,
	PA,
	PA,
	PA,
	PA,
	PA,
	PA,
	PA,
	
	NOT_A_PIN,	//VCC
	NOT_A_PIN,	//GND
	
	PF,
	PF,
	PF,
	PF,
	PF,
	PF,
	PF,
	PF,	
	
	NOT_A_PIN,	//REF
	NOT_A_PIN,	//GND
	NOT_A_PIN,	//AVCC pin 64
};
//_BOARD_AMBER128_
const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
	//PORTA
	NOT_A_PIN,
	NOT_A_PIN,
	_BV( 0 )	, // PD 
	_BV( 1 )	, // PD 
	_BV( 2 )	, // PD 
	_BV( 3 )	, // PD 
	_BV( 4 )	, // PD 
	_BV( 5 )	, // PD 
	_BV( 6 )	, // PD 
	_BV( 7 )	, // PD 
	//PORTB
	_BV( 0 )	, // PD
	_BV( 1 )	, // PD
	_BV( 2 )	, // PD
	_BV( 3 )	, // PD
	_BV( 4 )	, // PD
	_BV( 5 )	, // PD
	_BV( 6 )	, // PD
	_BV( 7 )	, // PD
	//PORTG
	_BV( 3 )	, // PD
	_BV( 4 )	, // PD
	
	NOT_A_PIN,
	NOT_A_PIN,
	NOT_A_PIN,
	NOT_A_PIN,
	NOT_A_PIN,
	
	//PORTD
	_BV( 0 )	, // PD
	_BV( 1 )	, // PD
	_BV( 2 )	, // PD
	_BV( 3 )	, // PD
	_BV( 4 )	, // PD
	_BV( 5 )	, // PD
	_BV( 6 )	, // PD
	_BV( 7 )	, // PD
	//PORTG
	_BV( 0 )	, // PD
	_BV( 1 )	, // PD
	//PORTC
	_BV( 0 )	, // PF 
	_BV( 1 )	, // PF 
	_BV( 2 )	, // PF 
	_BV( 3 )	, // PF 
	_BV( 4 )	, // PF 
	_BV( 5 )	, // PF 
	_BV( 6 )	, // PF 
	_BV( 7 )	, // PF
	//PORTG
	_BV( 2 )	,
	//PORTA
	_BV( 7 )	, // PF
	_BV( 6 )	, // PF
	_BV( 5 )	, // PF
	_BV( 4 )	, // PF
	_BV( 3 )	, // PF
	_BV( 2 )	, // PF
	_BV( 1 )	, // PF
	_BV( 0 )	, // PF
		
	NOT_A_PIN,
	NOT_A_PIN,
	
	//PORTF
	_BV( 7 )	, // PF
	_BV( 6 )	, // PF
	_BV( 5 )	, // PF
	_BV( 4 )	, // PF
	_BV( 3 )	, // PF
	_BV( 2 )	, // PF
	_BV( 1 )	, // PF
	_BV( 0 )	, // PF
	
	NOT_A_PIN,
	NOT_A_PIN,
	NOT_A_PIN,	
};

//TODO MAP TIMERS OC verkar indikera timerport!


const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	//PEN PIN
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	//PORTE
	NOT_ON_TIMER	,	
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	TIMER3A	,
	TIMER3B	,
	TIMER3C	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,

	//PORTB
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	TIMER0A			,
	TIMER1A			,
	TIMER1B			,
	TIMER2			,
	
	//PORTG
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	
	NOT_ON_TIMER	,	
	NOT_ON_TIMER	,	
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
	NOT_ON_TIMER	,
};