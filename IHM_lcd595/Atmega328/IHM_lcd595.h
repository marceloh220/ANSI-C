#ifndef IHM_LCD595_H
#define IHM_LCD595_H

/*************************************************************************************************************************
Nome: IHM_lcd595
Autor: Marcelo H Moraes
e-mail: marceloh220@hotmail.com

Descricao:
Biblioteca com funcoes para utilizacao de dispositivo LCD 16x2
Comunicacao Serial com deslocador de bits 74LS595
para controle de display utilizando 3 pinos de MCU.
Biblioteca Acontrolada, isto e, utilizavel para multiplas MCUs:
Intel MCS-51, AT89x51/52 com compilador SDCC.
Microchip PIC 12F, 16F ou 18F com compilador MikroC for PIC ou MPLAB XC8
Atmel ARV Atmega e Ttiny com Compilador AVR-GCC em IDE AVR/ATMEL Studio ou Code::Blocks
Plataforma de desenvolvimento Arduino com compilador AVR-GCC em IDE ARDUINO

Licensa:
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


Mapeamento do dispositivo adicione no inicio do codigo
//Para MCU MCS51, AT89S51/52 outos nao testados, compilador SDCC
  //Inclua no programa e altere conforme necessario
	#include <at89x51.h>
    #define MCS51
    #define LCD_OSC 		   12000000UL/12
  //Altere os pinos conforme necessario
    #define _clock             (P3_0=1)
    #define _Nclock            (P3_0=0)
    #define _data              (P3_1=1)
    #define _Ndata             (P3_1=0)
    #define _enable            (P3_2=1)
    #define _Nenable           (P3_2=0)

//Para MCU AVR, AVR Studio ou Code::Blocks
  //Inclua no programa e altere conforme necessario
    #include <avr/io.h>
	#define LCD_OSC 		   16000000UL
  //Altere os pinos conforme necessario
    #define _clock             (PORTD|=(1<<0))
    #define _Nclock            (PORTD&=~(1<<0))
    #define _data              (PORTD|=(1<<1))
    #define _Ndata             (PORTD&=~(1<<1))
    #define _enable            (PORTD|=(1<<2))
    #define _Nenable           (PORTD&=~(1<<2))
  //Direciona pinos como saidas
    #define _directclock       (DDRD|=(1<<0))
    #define _directdata        (DDRD|=(1<<1))
    #define _directenable      (DDRD|=(1<<2))

//Para MCU PIC, MPLAB XC8
//Inclua no programa e altere conforme necessario
  //Familia 16F
      #pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
	  #pragma config WDTE = ON        // Watchdog Timer Enable bit (WDT enabled)
	  #pragma config PWRTE = ON       // Power-Up Timer Enable bit (PWRT enabled)
	  #pragma config MCLRE = ON       // GP3/MCLR pin function select (GP3/MCLR pin function is MCLR)
	  #pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
	  #pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
	  #pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
	  #include <xc.h>
      #define LCD_OSC 		   	 4000000UL/4
    //Altere os pinos conforme necessario
      #define _clock             (RB0=1)
      #define _Nclock            (RB0=0)
      #define _data              (RB1=1)
      #define _Ndata             (RB1=0)
      #define _enable            (RB2=1)
      #define _Nenable           (RB2=0)
    //Direciona pinos como saidas
      #define _directclock       (TRISB0=0)
      #define _directdata        (TRISB1=0)
      #define _directenable      (TRISB2=0)

//Para MCU PIC, MikroC for PIC
//Inclua no programa e altere conforme necessario

  //Familia 12F
      #define LCD_OSC 		   	 4000000UL/4
    //Altere os pinos conforme necessario
      #define _clock             (gpio.b0=1)
      #define _Nclock            (gpio.b0=0)
      #define _data              (gpio.b1=1)
      #define _Ndata             (gpio.b1=0)
      #define _enable            (gpio.b2=1)
      #define _Nenable           (gpio.b2=0)
    //Direciona pinos como saidas
      #define _directclock       (trisio.b2=0)
      #define _directdata        (trisio.b2=0)
      #define _directenable      (trisio.b2=0)

  //Familia 16F
      #define LCD_OSC 		   	 4000000UL
    //Altere os pinos conforme necessario
      #define _clock             (RA0_bit=1)
      #define _Nclock            (RA0_bit=0)
      #define _data              (RA1_bit=1)
      #define _Ndata             (RA1_bit=0)
      #define _enable            (RA2_bit=1)
      #define _Nenable           (RA2_bit=0)
    //Direciona pinos como saidas
      #define _directclock       (trisa.b0=0)
      #define _directdata        (trisa.b1=0)
      #define _directenable      (trisa.b2=0)

//Para Arduino
  //Inclua no programa e altere conforme necessario
      #define LCD_OSC 		   	 16000000UL
    //Altere os pinos conforme necessario
      #define _clock             (digitalWrite(2,HIGH))
      #define _Nclock            (digitalWrite(2,LOW))
      #define _data              (digitalWrite(3,HIGH))
      #define _Ndata             (digitalWrite(3,LOW))
      #define _enable            (digitalWrite(4,HIGH))
      #define _Nenable           (digitalWrite(4,LOW))
    //Direciona pinos como saidas
      #define _directclock       (pinMode(2,OUTPUT))
      #define _directdata        (pinMode(3,OUTPUT))
      #define _directenable      (pinMode(4,OUTPUT))

//Para STM32F	(CORTEX-M)
//Inclua no programa e altere conforme necessario
		#include <stm32f1xx.h>
		#include <stm32f1xx_hal.h>
		#define CORTEX_M
		#define LCD_OSC 		   72000000U
	//Altere os pinos conforme necessario
		#define _clock             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)
		#define _Nclock            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)
		#define _data              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)
		#define _Ndata             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
		#define _enable            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
		#define _Nenable           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)



                     CI 74LS595
                       ___ ___
               1(Q1)  |       | 16(Vcc)
               2(Q2)  |       | 15(Q0)
               3(Q3)  |       | 14(DATA)
               4(Q4)  |       | 13(GND)
               5(Q5)  |       | 12(ENABLE)
               6(Q6)  |       | 11(CLOCK)
               7(Q7)  |       | 10(Vcc)
               8(GND) |_______| 09(Q7')

 *************************************************************************************************************************/

/* === Configuracoes do dispositivo === */

#include <avr/io.h>
#define LCD_OSC 		   16000000UL
//Altere os pinos conforme necessario
#define _clock             (PORTD|=(1<<0))
#define _Nclock            (PORTD&=~(1<<0))
#define _data              (PORTD|=(1<<1))
#define _Ndata             (PORTD&=~(1<<1))
#define _enable            (PORTD|=(1<<2))
#define _Nenable           (PORTD&=~(1<<2))
//Direciona pinos como saidas
#define _directclock       (DDRD|=(1<<0))
#define _directdata        (DDRD|=(1<<1))
#define _directenable      (DDRD|=(1<<2))

/* === Fim das configuracoes do dispositivo === */

/*Codigos para cursor do display, teste e enconte o melhor para o seu projeto*/
#define desligado 0x0C                  /*Cursor nao aparece*/
#define piscando  0x0D                  /*Cursor em bloco e piscando*/
#define ligado    0x0E                  /*Cursor em barra (ABC_)*/
#define alternado 0x0F                  /*Cursor alternando entre bloco e barra*/

/*Instrucao para background*/
#define on        1                     /*Luz de background ligada*/
#define off       0                     /*Luz de background desligada*/

void lcd_begin(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_home(void);
void lcd_cursor(unsigned char _modo);
void lcd_display(char mode);
void lcd_set(unsigned char _coluna, unsigned char _linha);
void lcd_backlight(unsigned char _on_off);
void lcd_printC(unsigned char _print);
void lcd_printS(const char *_print);
void lcd_printN(float _print, unsigned char _decimal);

#endif
