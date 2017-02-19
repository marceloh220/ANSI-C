/*
 * Marcelo H Moraes
 * marceloh220@hotmai.com
 * 
 * ADC library for atmega328/p
 */

#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED
#endif

#include <avr/interrupt.h>

void adc_init(char Vref, char prescale)    {
    
    //For Vref
    // 0 = Vref in pin Aref
    // 1 = Vref is the same of AVcc
    // 2 = No configuration, using 1
    // 3 = Vref int mcu of 1.1V
    
    
    //For prescale
    // 0 = 1/2
    // 1 = 1/2
    // 2 = 1/4
    // 3 = 1/8
    // 4 = 1/16
    // 5 = 1/32
    // 6 = 1/64
    // 7 = 1/128
    
    cli();
    if(Vref > 3 || Vref == 2) Vref = 1;
    ADMUX = 0;
    ADMUX |= (Vref<<6);
    ADCSRA = 0;
    ADCSRA |= (1<<ADEN);
    if(prescale > 7) prescale = 7;
    ADCSRA |= prescale;
                                           }

int adc_read(char pin)                     {
    if (pin > 7) return 0;
    DIDR0 |= (1<<pin);
    ADMUX &= 0xF0;
    ADMUX |= pin;
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    return (ADC);

                                           }

void acd_digital(char pin)                 {
    DIDR0 &= ~(1<<pin);
                                           }
