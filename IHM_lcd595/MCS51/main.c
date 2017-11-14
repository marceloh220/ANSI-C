/*
	Marcelo H Moraes
	marceloh220@hotmail.com
	MCU: AT89S51
	Clock: 12MHz/12 (1us machine cycle)
	Compiler: SDCC (3.6.0 Linux)
	Just display in device IHM_lcd595.
*/

#include <at89x51.h>
#include "delay.h"
#include "IHM_lcd595.h"

int count = 0;

int main() {

	lcd_begin();
	lcd_backlight(on);
	lcd_set(0,0);
	lcd_printS("Hello World!");
	for(;;) {				/*THE LOOP*/
		lcd_set(0,1);
		lcd_printN(count++,0);
		lcd_printS("     ");
		P0_0 = 0;
		_delay_ms(500);
		P0_0 = 1;
		_delay_ms(500);
		lcd_set(0,1);
	}

}
