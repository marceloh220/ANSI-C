#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED
#endif

#include <avr/interrupt.h>

void adc_init(char Vref, char prescale)    {
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
