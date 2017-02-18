/*
	Marcelo H Moraes
	marceloh220@hotmail.com
	MCU: AT89S51
	Clock: 12MHz/12 (1us machine cycle)
	Compiler: SDCC (3.6.0 Linux)
	Just display in device IHM_lcd595.
*/

#include <at89x51.h>
//Altere os pinos conforme necessario
    #define _clock             (P3_0=1)
    #define _Nclock            (P3_0=0)
    #define _data              (P3_1=1)
    #define _Ndata             (P3_1=0)
    #define _enable            (P3_2=1)
    #define _Nenable           (P3_2=0)
    #define __delay            _delay_ms(40)
    #define __delay_send       __asm	nop __endasm
    #define MCS51	1
  //Biblioteca
    #include "delay.h"
    #include "IHM_lcd595.h"

int main() {

	lcd_begin();
	lcd_background(on);
	lcd_printS("ABCDEFGHIJKLMNOP");
	lcd_set(0,1);
	lcd_printS("QRSTUVWXYZ012345");
	for(;;) {				/*THE LOOP*/
		/*some other codes*/
	}

}
