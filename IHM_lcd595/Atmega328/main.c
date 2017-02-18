/*
 
Marcelo H Moraes
marceloh220@hotmail.com

avr-gcc (gcc version 6.3.0)

MCU: atmega328
Clock: internal RC 8MHz

Fuses:	low = 0xe2		(8MHz internal RC, slow rise 62ms)
		high = 0xde		(take care with high fuses!)
		extend = 0xfd	(BOD 2.7V)
		(i don't care about lock bits)

Measure distance with ultrason HC-SR04.

Pulse width with hardware capture in timer1
Echo : 	PB0
Triger:	PD7

Display with IHM_LCD595 device
Clock:	D2
Data:	D3
Enable:	D4

Oh yeeeeh, only 5 pins.
Is possible do that even in family attiny DIP-8!


Licence:
 
The MIT License (MIT)

Copyright (c) 2016 Marcelo Henrique Moraes Dermones

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 

*/
    
    #define  F_CPU 8000000UL
    #include <avr/io.h>
    #include <util/delay.h>
	#include <stdint.h>
	#include <avr/interrupt.h>
    #define _clock             (PORTD|=(1<<2))
    #define _Nclock            (PORTD&=~(1<<2))
    #define _data              (PORTD|=(1<<3))
    #define _Ndata             (PORTD&=~(1<<3))
    #define _enable            (PORTD|=(1<<4))
    #define _Nenable           (PORTD&=~(1<<4))
    #define _directclock       (DDRD|=(1<<2))
    #define _directdata        (DDRD|=(1<<3))
    #define _directenable      (DDRD|=(1<<4))
    #define __delay            _delay_us(37)
    #define __delay_send       _delay_us(1)
    #include "IHM_lcd595.h"
	
	uint8_t ovf_capture;
	uint32_t pulse;

	ISR(TIMER1_OVF_vect) {
		ovf_capture++;

	}
	
	ISR(TIMER1_CAPT_vect) {
		TCNT1 = 0;
		if(TCCR1B&(1<<ICES1)) {
			TCCR1B|=3;
			TCCR1B&=~(1<<ICES1);
			pulse=0;
		}
		else {
			TCCR1B=0;
			TIMSK1=0;
			pulse=(1024*ovf_capture+ICR1);
			ovf_capture = 0;
		}
	}

	double measure_distance() {
		cli();
		TCCR1B=(1<<ICNC1)|(1<<ICES1);
		TIMSK1=(1<<ICIE1)|(1<<TOIE1);
		sei();
		PORTD|=(1<<7);
		_delay_us(15);
		PORTD&=~(1<<7);
		while(TCCR1B);
		return pulse*0.1359; //For different clock or unit this number must be recalculated
	}


	int main() {
	
	lcd_begin();
	lcd_background(on);
	DDRD|=(1<<7);
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
