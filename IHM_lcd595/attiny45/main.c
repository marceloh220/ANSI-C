/*
 
Marcelo H Moraes
marceloh220@hotmail.com

MCU: attiny45
Clock: internal RC 8MHz

Fuses:	low = 0x62		(8MHz internal RC, slow rise 62ms)
	high = 0xde	(BOD 2.7V, take care with bits RSTDISBL and SPIEN)
	extend = 0xff	(i don't know what self-programming is, let then quiet)	
	(i don't care about lock bits)

Measure distance with ultrason HC-SR04.
Yep, with a simple DIP-8 mcu attiny45 using 5 pins.

Pulse width with hardware capture in timer1
Echo : 	PB3
Triger:	PB2

Display with IHM_LCD595 device
Clock:	PB4
Data:	PB1
Enable:	PB0

*/


#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define _clock			(PORTB|=(1<<4))
#define _Nclock			(PORTB&=~(1<<4))
#define _data			(PORTB|=(1<<1))
#define _Ndata			(PORTB&=~(1<<1))
#define _enable			(PORTB|=(1<<0))
#define _Nenable		(PORTB&=~(1<<0))
#define _directclock		(DDRB|=(1<<4))
#define _directdata		(DDRB|=(1<<1))
#define _directenable		(DDRB|=(1<<0))
#define __delay			_delay_us(40)
#define __delay_send		_delay_us(1)
#include "IHM_lcd595.h"

uint8_t	ovf_capture;
uint32_t pulse;

ISR(TIMER0_OVF_vect) {
	ovf_capture++;
}

ISR(INT0_vect) {
	if(MCUCR&(1<<ISC00)) {
		TCCR0B=3;
		MCUCR&=~(1<<ISC00);
	}
	else {
		TCCR0B=0;
		GIMSK=0;
		MCUCR&=~(1<<ISC01);
		pulse=255*ovf_capture+TCNT0;
	}
}

float measure_distance() {
	cli();
	DDRB|=(1<<3);
	TCNT0=0;
	ovf_capture=0;
	MCUCR|=3;
	TIMSK=(1<<TOIE0);
	GIMSK=(1<<INT0);
	sei();
	PORTB|=(1<<3);
	_delay_us(10);
	PORTB&=~(1<<3);
	while(GIMSK);
	return pulse*0.1359;
}

int main() {		
	lcd_begin();
	lcd_background(on);
	lcd_set(0,0);
	lcd_printS("Distance");

	while(1) {
		lcd_set(0,1);
		lcd_printN(measure_distance(),2);
		lcd_printS(" cm      ");
		_delay_ms(500);
	}
	return 0;
}
