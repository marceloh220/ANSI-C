#ifndef BITS16_H
#define BITS16_H
#endif

/************************************************************************************************************************

Nome: Biblioteca(livro) para o dispositivo bit16
Autor: Marcelo H Moraes
e-mail: marceloh220@hotmail.com
Em caso de problemas, bugs, duvidas ou dicas de melhorias, sintam-se avontade para entrar em contato por email.
PS: So tenho especialidade nos microcontroladores citados neste documento, talvez nao possa ajudar com arquiteturas muito diferente

Descricao:
Biblioteca com funcoes para utilizacao de dispositivo expansor de 16 bits mais dois display 7-seg
Comunicacao Serial com deslocador de bits 74LS595 para controle de display utilizando 2 pinos de MCU (Clock,Data)

O dispositivo possui 16 pinos que podem ser ligados de desligados individualmente com a funcao "bits16_out()"
Ex: 	bits16_out(PO0,on);		//Pino PO0 ligado
	bits16_out(PO1,off);		//Pino PO1 desligado
	
Alem de possuir dois displays de 7 seguimentos que podem demonstrar numeros de 0 ate 99 com a funcao "bit16_print()"
EX:	bits16_print(99);		//Mostra no display numero 99

Os pontos do display podem ser ligados com a funcao "bits16_dot()" e deve ser enviada apos o envio de um numero
Ex:	bits16_print(99);		//Mostra no display numero 99
	bits16_dot(unit);		//liga o ponto na unidade ficando 99.
	ou mesmo
	bits16_print(9);		//Mostra no display numero 09
	bits16_dot(tem);		//liga o ponto na dezena ficando 0.9

Pra uso dos pinos recomendado desativar o display atravez de switch ou de jump de selecao

Biblioteca Acontrolada, isto e, utilizavel para multiplas MCUs:
Intel MCS-51, At89x51/52/53 com compilador SDCC
Microchip PIC 12F, 16F ou 18F com MikroC for PIC ou MPLAB IDE/XC8
Atmel ARV ATxmega, Atmega e ATtiny com Compilador AVR-GCC
Plataforma de desenvolvimento Arduino com Arduino IDE

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


//Para MCU AT89S51, compilador SDCC
  //Inclua no programa e altere conforme necessario
#include <at89x51.h>
#define _data	(P3_2 = 1)
#define _Ndata	(P3_2 = 0)
#define _clock	(P3_3 = 1)
#define _Nclock	(P3_3 = 0)
#define MCS51	1		//compilador ignora funcao de init
#include "bits16.h"
//MCS-51 nao precisa de direcionamento de pinos, funcao ""bits16_init()" nao deve ser utilizada


//Para MCU AVR, AVR Studio, Code::Blocks ou Arduino IDE
  //Inclua no programa e altere conforme necessario
    #define  F_CPU 16000000UL
    #include <avr/io.h>
  //Altere os pinos conforme necessario
    #define _clock             (PORTD|=(1<<0))
    #define _Nclock            (PORTD&=~(1<<0))
    #define _data              (PORTD|=(1<<1))
    #define _Ndata             (PORTD&=~(1<<1))
  //Direciona pinos como saidas
    #define _directclock       (DDRD|=(1<<0))
    #define _directdata        (DDRD|=(1<<1))
  //Biblioteca
    #include "16bits.h"


//Para MCU PIC, MikroC for PIC
//Inclua no programa e altere conforme necessario

  //Familia 12F
    //Altere os pinos conforme necessario
      #define _clock             (gpio.b0=1)
      #define _Nclock            (gpio.b0=0)
      #define _data              (gpio.b1=1)
      #define _Ndata             (gpio.b1=0)
    //Direciona pinos como saidas
      #define _directclock       (trisio.b2=0)
      #define _directdata        (trisio.b2=0)
    //Biblioteca
      #include "16bits.h"

  //Familia 16F
    //Altere os pinos conforme necessario
      #define _clock             (RA0_bit=1)
      #define _Nclock            (RA0_bit=0)
      #define _data              (RA1_bit=1)
      #define _Ndata             (RA1_bit=0)
    //Direciona pinos como saidas
      #define _directclock       (trisa.b0=0)
      #define _directdata        (trisa.b1=0)
    //Biblioteca
      #include "16bits.h"

//Para MCU PIC, MPLAB XC8
    //Familia 16F
    //Altere os pinos conforme necessario
      #define _clock             (RB0=1)
      #define _Nclock            (RB0=0)
      #define _data              (RB1=1)
      #define _Ndata             (RB1=0)
    //Direciona pinos como saidas
      #define _directclock       (TRISB0=0)
      #define _directdata        (TRISB1=0)
    //Biblioteca
      #include "16bits.h"


//Para Arduino
  //Inclua no programa e altere conforme necessario
    //Altere os pinos conforme necessario
      #define _clock             (digitalWrite(2,HIGH))
      #define _Nclock            (digitalWrite(2,LOW))
      #define _data              (digitalWrite(3,HIGH))
      #define _Ndata             (digitalWrite(3,LOW))
    //Direciona pinos como saidas
      #define _directclock       (pinMode(2,OUTPUT))
      #define _directdata        (pinMode(3,OUTPUT))
    //Biblioteca
      #include <16bits.h>


Testado com dispositivos acima, software escrito em C-ANSI padrão C89, alta compatibilidade

*************************************************************************************************************************/

/*Nome para os pinos de saida (PO0 ate PO15)*/
#define PO0 	0
#define PO1 	1
#define PO2 	2
#define PO3 	3
#define PO4 	4
#define PO5 	5
#define PO6 	6
#define PO7 	7
#define PO8 	8
#define PO9 	9
#define PO10 	10
#define PO11 	11
#define PO12 	12
#define PO13 	13
#define PO14 	14
#define PO15 	15

/*Nomes para estado dos pinos de saida*/
#ifndef on
#define on 1
#endif
#ifndef off
#define off 0
#endif
/*Escolha o estado entre ligado (on) e desligado (off)*/

/*Para ligar o ponto do display de 7 seguimentos da placa*/
#ifndef ten
#define ten 1
#endif
#ifndef unit
#define unit 0
#endif
/*Escolha ligar ponto da dezena (ten) ou da unidade (unit)*/

/*Vetor para os seguimentos do display.
EX: Ao enviar 0 para o display, sera enviado 0xC0, no display de anodo comum isto representa os seguimentos abcdef ligados e g e o ponto desligado.
Em microcontroladores  8051/pic e similares este vetor ficara apenas na memoria de programa.
Isto facilita o acesso e economiza memoria de dados.
Porem nos AVR ele  ficara na memoria de dados.
Isto ocorre porque os AVRs utilizam diretivas PROGMEM e prog_read para realizar acesso aos dados da memoria de programa com instrucao LPM.
Aqui sera mantido dessa forma para manter a compatibilidade.
Verifique sobre PROGMEM e modifique os acessos para melhorar o programa para MCU AVR.*/
const char seguimentos[10]={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};

/*Guarda o estado das saidas do expansor de 16bits.
Diretiva static faz com que variavel fique dispinivel apenas para a biblioteca.
E como se fosse um atributo private em programacao otientada a objetos.*/
static int ___ports;

/*Mais uma vez a diretiva static fazendo com que esta funcao fique utilizada apenas pela biblioteca impedindo o uso dela em outros arquivsos .c
E como um metodo private em programacao orientada a objetos.
Esta funcao recebe um dado de 16 bits e envia de forma serial atravez dos pinos selecionados como data e clock, caracterizando assim uma comunicacao serial sincrona.*/
static void bits16_send(int __dado) {	/*Envia os 16bits para o dispositivo*/
    unsigned char __aux;		/*Variavel de contagem dos envios*/
    for(__aux=0; __aux<16; __aux++) {	/*Loop de 16 ciclos para enviar os 16bits*/
      if(__dado&0x01) _data;		/*Se o bit 0 for 1, pino de dados vai pra um*/
      else  _Ndata;			/*Se o bit 0 for 0, pino de dados vai pra zero*/
      _clock;				/*Pulso de clock em 1*/
      _Nclock;				/*Pulso de clock em 0*/
      __dado=__dado>>1;			/*Desloca o dado para enviar o proximo bit*/
	}
}

/*Funcao de inicializacao do dispositivo.
Coloca os pinos de data e clock como saida, definindo MCS51 como 1, esta funcao e ignorada  pois os MCU MCS51 nao precisam de direcionamento de pinos.*/
#ifndef MCS51
void bits16_init() {			/*Em MCU que nao sejam MCS51, esta funcao deve ser a primeira a ser chamada*/
	_directclock;			/*Pino de clock como saida*/
	_directdata;			/*Pino de data como saida*/
}
#endif

/*Esta funcao liga e desliga os pinos de saida individualmente.
Pode ser enviado um comando para ligar um pino "bits16_out(PO15,on);" ou desligar outro pino "bits16_out(PO5,off);" sem alterar o estado dos outros.*/
int bits16_out(unsigned char __port, unsigned char __state) {
	if(__state)			/*Se estado on*/
		___ports|=(1<<__port);	/*Liga o pino selecionado*/
	else				/*Se nao, estado off*/
		___ports&=~(1<<__port);	/*Desliga o pino selecionado*/
	bits16_send(___ports);		/*Envia os pinos para o dispositivo*/
	return ___ports;		/*Retorna estado atual dos pinos, o que pode ser utilizado de varias formas*/
}

/*Envia numeros de 0 a 99 para o display.
Esta funcao sepada as dezenas das unidades e envia para os display de 7 seguimentos um numero de 0 a 99.
Para enviar o numero 01 basta utilizar "bits16_print(1);", ou para enviar o numero 86 "bits16_print(86)".
Em um exemplo melhor pode se enviar uma variavel que vai de 0 ate 99 para o display.
while(1) {
	char count = 0;
	count++;
	if(count<99)
		bits16_print(count);
	_delay_ms(500);
}
*/
void bits16_print(int __number) {
	int __send;					/*Variavel que organiza os numeros a serem enviados*/
	__send = seguimentos[(int)__number/10]<<8;	/*Salva as dezenas nos 8 bits mais significativo*/
	__send |= seguimentos[(int)__number%10];	/*Soma com unidade nos 8 bits menos significativos*/
	bits16_send(__send);				/*Envia unidades e dezenas para o display*/
	___ports=__send;				/*Salva nos ports os caracteres enviados*/
}

/*Liga os pontos do display de 7-seguimentos.
"bits16_dot(unit);" liga o ponto do primeiro display.
"bits16_dot(ten);" liga o ponto do segundo display.
Esta funcao deve ser chamada assim que um numero e enviado para os displays.
Em um exemplo mais detalhado, a funcao abaixo conta de 0.0 ate 0.9 em decimais, apos isto conta de 1 ate 99 em dezenas.
while(1) {
	char count = 0;
	if(count<10) {
		count++;
		bits16_print(count);
		bits16_dot(ten);
	}
	else if(count<999) {
		count+=10;
		bits16_print(count/10);
	}
	_delay_ms(500);
}*/
void bits16_dot(char __unit) {
	if(__unit)			/*Se enviado ponto do display de unidade*/
		___ports&=~(1<<15);	/*Envia para o display os seguimentos com este ponto ligado*/
	else				/*Se nao, enviado ponto do display de dezena*/
		___ports&=~(1<<7);	/*Envia para o display os seguimentos com este ponto ligado*/
	bits16_send(___ports);		/*Envia os dados para o dispositivo*/
}
