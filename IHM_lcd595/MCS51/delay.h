#ifndef DELAY_H
#define DELAY_H
#endif

/*

Marcelo H Moraes
marceloh220@hotmail.com 
Use timer0 for generate somes delays

ONLY FOR MCS51
Tested only in AT89S51 and AT89S52
Use at your own risc!

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

You have been warned!

 */

void _delay_ms(int ms) {
	TR0 = 0;
	TF0 = 0;
	TMOD |= 1;
	while(ms) {
		TL0 = 0x18;
		TH0 = 0xFC;
		TR0 = 1;
		ms--;
		while(!TF0);
		TR0 = 0;
		TF0 = 0;
	}
}

/*Can be unseless for some models, but, there is*/
void _delay_us(int ms) {
	TR0 = 0;
	TF0 = 0;
	TL0 = (255-(ms&0x0F));
	TH0 = (255-(ms>>8));
	TR0 = 1;
	while(!TF0);
	TR0 = 0;
	TF0 = 0;
}
