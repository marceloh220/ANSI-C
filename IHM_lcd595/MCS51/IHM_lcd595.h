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
    #define MCS51	1
  //Altere os pinos conforme necessario
    #define _clock             (P3_0=1)
    #define _Nclock            (P3_0=0)
    #define _data              (P3_1=1)
    #define _Ndata             (P3_1=0)
    #define _enable            (P3_2=1)
    #define _Nenable           (P3_2=0)
    #define __delay            _delay_ms(40)
    #define __delay_send       __asm	nop __endasm
  //Biblioteca
    #include "delay.h"
    #include "IHM_lcd595.h"

//Para MCU AVR, AVR Studio ou Code::Blocks
  //Inclua no programa e altere conforme necessario
    #define  F_CPU 16000000UL       //Altere para velocidade da cpu
    #include <avr/io.h>
    #include <util/delay.h>
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
    #define __delay            _delay_us(40)
    #define __delay_send       _delay_us(1)
  //Biblioteca
    #include "IHM_lcd595.h"

//Para MCU PIC, MPLAB XC8
//Inclua no programa e altere conforme necessario
  //Familia 16F
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
      #define __delay            __delay_us(40)
      #define __delay_send       __delay_us(1)
    //Biblioteca
      #include "IHM_lcd595.h"

//Para MCU PIC, MikroC for PIC
//Inclua no programa e altere conforme necessario

  //Familia 12F
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
      #define __delay            Delay_us(40)
      #define __delay_send       Delay_us(1)
    //Biblioteca
      #include "IHM_lcd595.h"

  //Familia 16F
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
      #define __delay            Delay_us(40)
      #define __delay_send       Delay_us(1)
    //Biblioteca
      #include "IHM_lcd595.h"

//Para Arduino
  //Inclua no programa e altere conforme necessario
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
      #define __delay            delayMicrosecond(40)
      #define __delay_send       delayMicrosecond(1)
    //Biblioteca
      #include <IHM_lcd595.h>



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

#endif


/*Codigos para cursor do display, teste e enconte o melhor para o seu projeto*/
  #define desligado 0x0C                  /*Cursor nao aparece*/
  #define piscando  0x0D                  /*Cursor em bloco e piscando*/
  #define ligado    0x0E                  /*Cursor em barra (ABC_)*/
  #define alternado 0x0F                  /*Cursor alternando entre bloco e barra*/

/*Instrucao para background*/
  #define on        1                     /*Luz de background ligada*/
  #define off       0                     /*Luz de background desligada*/
  #define dado      0                     /*Enviando dado para display*/
  #define comando   1                     /*Enviando comando para display*/

  char ___background;                     /*Variavel que guarda se luz de background esta ligada ou desligada*/


/* === Funcoes basicas de envio de dados para o dispositivo com o CI 74LS595 ===*/

  /*Funcao com delay extra de 2ms para instrucoes que precisam de mais tempo para ser excultadas*/
  static void ___delay_init() {

    unsigned char _countdelay;            /*Variavel auxiliar de contagem*/

    for (_countdelay = 0; _countdelay < 50; _countdelay++)
      __delay;                            /* Funcao delay de 40us execultada 50 vezes*/
                                          /* 40us * 50 = 2000us = 2ms*/

  }

  /*Funcao que envia 5 bits para o deslocador 74LS595
    Parametro _dado e o dado que sera enviado para o CI 74LS595*/
  static void lcd_send(unsigned char _dado) {

    unsigned char _aux;                   /*Variavel auxiliar de contagem*/

    for(_aux=0; _aux<6; _aux++){          /*Envia dado bit a bit para display ate o sexto bit*/

      if(_dado&0x01) _data;               /*Verifica se dado a ser enviado e 1 comparando com mascara 0b00000001*/
                                          /* Operacao & (AND) realizada equivale a:*/
                                          /*  _dado = 0bxxBR dddA (A = bit a ser analizado)*/
                                          /*        & 0b0000 0001*/
                                          /* Se o resultado da operacao for 1, o que ocorre quando o bit A = 1*/
                                          /* Seta o pino escolhido como _data e envia 1 para deslocador de bit 74LS595*/

      else  _Ndata;                       /*Se nao, limpa o pino escolhido como _data e envia 0 para deslocador*/
                                          /*de bit 74LS595*/

      _clock;                             /*Borda de subida para deslocar dado enviado no deslocador de bit 74LS595*/
      __delay_send;                       /*Aguarda o bit ser deslocado no 74LS595*/
      _Nclock;                            /*Borda de descida para deslocar dado enviado no deslocador de bit 74LS595*/

      _dado = _dado>>1;                   /*Desloca dado uma casa para a direita para enviar o proximo bit*/
                                          /* _dado = 0b0xxB RddA*/

    }                                     /*Fim do envio dos 6bits*/
	_enable;							  /*Borda de subida para o sinal de enable*/
	__delay_send;						  /*Aguarda tempo para reconhecimento de borda*/
	_Nenable;							  /*Borda de descida para o sinal de enable*/
    __delay;                              /*Aguarda tempo para LCD aceitar instrucao*/

  }

  /*Funcao que organiza o dado a ser enviado*/
  static void lcd_d_c(unsigned char _dado, unsigned char _comando) {
                                          /*Parametro _dado e o dado que sera enviado para o display
                                            Parametro comando determina se sera enviado para o display dado ou comando
                                            _comando = 0;  envinado dado
                                            _comando = 1;  enviando comando*/
    unsigned char _enviar;                /*Variavel de organizacao de dado*/

    _enviar = _dado>>4;                   /*Salva na variavel o nibble mais significativo do dado
                                            deslocado 4 casas para direita
                                            __dado   = 0bDDDD dddd;  (D = mais significativo; d = menos significativo)
                                            _enviado = 0b0000 DDDD;  (D = dado do nibble mais significativo)*/

    if (_comando) _enviar &= ~(1<<4);     /*Se parametro comando for 1, altera o 5 bit para 0*/
    else _enviar|=(1<<4);                 /*Se nao, se parametro comando for 0, altera o 5 bit para 1*/
                                          /*_enviado = 0b000R DDDD; (R = bit de RW)*/

    if (___background) _enviar |= (1<<5); /*Se background do display setado como ligado, altera o 6 bit para 1*/
    else _enviar &= ~(1<<5);              /*Senao, altera o 6 bit para 0*/
                                          /*_enviado = 0b00BR DDDD; (B = bit de Background)*/

    lcd_send(_enviar);                    /*_enviado = 0b00BR DDDD; envia os 4 bits mais significativos para o display
                                            mais o bit B de backgorund e o bit R de RW*/

    _enviar=_dado;                        /*Salva na variavel enviar o nibble menos significativo do dado
                                            __dado   = 0bDDDD dddd;
                                            _enviado = 0bxxxx dddd;
                                            x = bits nulos, so serao utilizados no segundo envio os 4 primeiros bits*/

    if (_comando) _enviar &= ~(1<<4);     /*Se parametro comando for 1, altera o 5 bit para 0*/
    else _enviar|=(1<<4);                 /*Se nao, se parametro comando for 0, altera o 5 bit para 1*/
                                          /*_enviado = 0bxxxR dddd; (R = bit de RW)*/

    if (___background) _enviar |= (1<<5); /*Se background do display setado como ligado altera o 6 bit para 1*/
    else _enviar &= ~(1<<5);              /*Senao, altera o 6 bit para 0
                                            _enviado = 0bxxBR dddd; (B = bit de Background)*/

    lcd_send(_enviar);                    /*_enviado = 0bxxBR dddd; envia os 4 bits menos significativos para o display
                                            mais o bit B de backgorund e o bit R de RW*/

  }


/* === Funcoes de instrucoes basicas do display LCD === */

  /*Funcao de inicializacao do display 16x2, modo 4 bits, duas linhas e matrix 5x8*/
  void lcd_begin(void) {
	_Nclock;
	_Ndata;
	_Nenable;
#ifndef MCS51
    _directclock;                         /*Define pino de clock  como saida*/
    _directdata;                          /*Define pino de dado   como saida*/
	_directenable;						  /*Define pino de enable como saida*/
#endif

	lcd_d_c(0x03,comando);
	___delay_init();
	lcd_d_c(0x03,comando);
	___delay_init();
	lcd_d_c(0x03,comando);				  /*Tres tentativas para resolver bugs*/
	___delay_init();
	
    lcd_d_c(0x02,comando);                /*Envia comando para colocar lcd em modo de 4bits*/
    ___delay_init();                      /*Aguarda tempo de 2ms para inicializacao
                                            RW  D7-D4
                                            0  0010*/


    lcd_d_c(0x28,comando);                /*Envia comando para configurar lcd como 2 linhas e matriz 5x8*/
    ___delay_init();                      /*Aguarda tempo de 2ms para inicializacao
                                            RW  D7-D4
                                             0  0010
                                             0  1000*/

    lcd_d_c(desligado,comando);           /*Envia comando para configurar cursor desligado como padrao
                                            RW  D7-D4
                                             0  0000
                                             0  1100*/
    lcd_d_c(0x01,comando);				  /*Garante que display esteja limpo a inicializar*/

  }

  /*Funcao de limpar o lcd e retornar cursor para linha 1 coluna 0*/
  void lcd_clear(void) {

    lcd_d_c(0x01,comando);                /*Envia comando para limpar o display lcd*/
    ___delay_init();                      /*Aguarda tempo de 2ms para a instrucao ser executada
                                             RW  D7-D4
                                              0  0000
                                              0  0001*/

  }

  /*Funcao de retornar cursor para primeira linha e primeira coluna sem limpar o display*/
  void lcd_home(void) {

    lcd_d_c(0x02,comando);                /*Envia comando para retornar o cursor do display para o inicio*/
    ___delay_init();                      /*Aguarda tempo de 2ms para a instrucao ser executada
                                            RW  D7-D4
                                             0  0000
                                             0  0010*/

  }

  /*Funcao de escolher o modo do cursor*/
  void lcd_cursor(unsigned char _modo) {  /*Parametro _modo e a forma com que o cursor do display ira se comportar
                                            Parametros aceitos sao:
                                             desligado = Cursor nao aparece
                                             piscando  = Cursor em bloco e piscando
                                             ligado    = Cursor em barra (ABC_)
                                             alternado = Cursor alternando entre bloco e barra*/

    lcd_d_c(_modo,comando);               /*Envia comando para mudar o modo do display*/

  }

  /*Funcao de desligar o display, esta funcao nao apaga o que estiver escrito
    ao ligar o display novamente ele volta a apresentar o que estava escrito*/
  void lcd_display(char mode) {
	if(mode)
		lcd_d_c(0x08,comando);            /*Envia comando para desligar o display
                                            RW  D7-D4
                                             0  0000
                                             0  1000*/
    else lcd_d_c(0x0C,comando);           /*Envia comando para ligar o display
                                            RW  D7-D4
                                             0  0000
                                             0  1100*/

  }

  /*Funcao para posicionar o cursor do display LCD*/
  void lcd_set(unsigned char _coluna, unsigned char _linha) {
                                          /*Parametro _linha determina qual linha o cursor deve se posicionar
                                             _linha = 0; Primeira linha do display LCD
                                             _linha = 1; Segunda linha do display LCD
                                            Parametro _coluna determina em qual coluna o cursor deve se posicionar
                                            Valores aceitos de 0 ate 15
                                             0 = primeira coluna; 1 = segunda coluna; ...;
                                            15 = decima sexta coluna do display LCD
                                            Cuidados:
                                            Posicoes invalidas farao com que o cursor fique posicionado fora do display*/

    unsigned char _aux;                   /*Variavel que auxilia no posicionamento*/

    if(_linha) _aux = 0xC0;               /*Se parametro _linha for 1 envia o endereco da segunda linha do display
                                            (0xCX; X = coluna)*/
    else _aux = 0x80;                     /*Se nao, quando envia 0 para parametro
                                            Envia endereco da primeira linha do display (0x8X; X = coluna)*/

    _aux |= _coluna;                      /*Realiza uma operacao | (OR) com o nibble de coluna
                                             _aux = 0bLLLL 0000  (L = parametro linha; pode ser C=0b1100 ou 8=0b1000)
                                                  | 0b0000 CCCC  (C = parametro coluna; deve ser de 0=0b0000 a 15=0b1111
                                                  = 0bLLLL CCCC
                                            Esta operacao resulta na alteracao do nibble menos significativo (de coluna)
                                            Sem alterar o nibble mais significativo (de linha)*/

    lcd_d_c(_aux,comando);                /*Envia o comando para para posicionar o cursor do display no endereco*/
    ___delay_init();                      /*Aguarda o display realizar o posicionamento
                                            Um tempo de 2ms garante que o cursor tenha tempo de se posiionar*/

  }

  /*Funcao que liga ou desliga o background do display LCD*/
  void lcd_background(unsigned char _on_off) {
                                          /*Parametro _on_off determina o status da luz de backgroud
                                            Parametros aceitos:
                                             on  = luz de background ligada
                                             off = luz de background desligada*/

    ___background = _on_off;              /*Seta a variavel de controle de luz do background*/
    lcd_d_c(0x00,comando);                /*Envia para o display um comando NOP (0x00)
                                            Esta instrucao faz com que o LCD faca nada
                                            Dessa forma o bit de background no CI 74LS595 seja atualizado
                                            sem modificacoes indesejadas no display*/

  }


/* === Funcoes de escrita de caracteres e strings no display ===*/

  /*Funcao para imprimir caracteres(C), simplesmente envia um caracter para o display*/
  void lcd_printC(unsigned char _print) {
                                          /*Parametro _print e o caracter que sera enviado ao display LCD
                                            Parametro do tipo char enviado conforme a tabela ASCII
                                            O Compiador C reliza a convercao quando se coloca o caracter entre apostrofos
                                            _print = 'a'; imprime o caracter 'a' que na tabela ASCII e o numero 97*/

    lcd_d_c(_print,dado);                 /*Envia o parametro _print para o display como um dado*/

  }

  /*Funcao para imprimir strings(S) no display*/
  void lcd_printS(const char *_print) {
                                          /*Parametro *_print e um ponteiro, ela aponta pra o endereco de uma variavel
                                            Trataremos os enderecos para qual este ponteiro aponta como uma string
                                            Que e um conjunto de variaveis do tipo char que formam uma cadeia de carateres
                                            Importante que ela seja acompanhada pelo modificador const
                                            Para que o MCU procure os caracteres direto na memoria de programa
                                            O compilador C realiza a criacao de um conjunto de caracteres
                                            quando os colocamos entre aspas
                                             _print[] = "ABCD";
                                            Isto indica um conjunto de variareis do tipo char com 5 enderecos
                                            o que equivale a *_print[] = "ABCD";
                                            Onde *_print e o dado salvo e _print e o endereco onde o dado esta guardado
                                            Podemos representar este tipo de variavel como:
                                                              *_print[_print+0] = 'A';
                                                              *_print[_print+1] = 'B';
                                                              *_print[_print+2] = 'C';
                                                              *_print[_print+3] = 'D';
                                                              *_print[_print+4] = '\0';
                                             '\0' e um caracter nulo que representa o numero decimal 0
                                             Este caracter e colocado pelo compilador C para indicar o final de uma string
                                             Em um resumo breve entendamos:
                                              _print e algum endereco da memoria
                                              *_print e o dado que esta guardado no endereco para o qual _print aponta
                                            Avancando _print avancamos o endereco
                                            Com *_print requisitamos o que estiver salvo no endereco*/

    while(*_print!='\0') {                /*Enquanto o programa nao encontra um caracter nulo(0),
                                            Envia os caracteres contidos na string _print*/
      lcd_d_c(*_print,dado);              /*Envia para o display LCD o conteudo do endereco apontado pelo ponteiro*/
      _print++;                           /*Avanca o ponteiro para a proxima posicao da string*/

    }

  }

/* === Funcoes de envio de numeros inteiros ou deimais para o display ===
       Os numeros nao sao impressos corretamente no display LCD, pois este so aceita caracteres da tabela ASCII
       Esta funcao serve para converter os numeros inteiros ou decimais em seus caracteres correspondentes
       Desta forma e possivel enviar para o display LCD o valor inteiro lido pelo conversor ADC do MCU
       ou um resultado inteiro ou decimal de alguma operacao matematica
       este metodo funciona bem para numeros de 1e6 ou ate 10e-8 positivos ou negativos*/

  void lcd_printN(float _print, unsigned char _decimal) {
                                                    /*Parametro _print pode receber um numero inteiro(2016)
                                                       ou um numero com ponto flutuante (decimal 3.14)
                                                      Parametro _decimal determina o numero de casas apos a "virgula"
                                                      Lembre-se
                                                       C ANSI e uma implementacao norte americana
                                                       por isso casas decimais sao separadas por ponto e nao por virgula*/

          unsigned char _count;                     /*Variavel utilizada para contagens necessarias para o algoritimo*/
          volatile long int _aux;                   /*Variavel que recebe os numeros conforme eles sao processados*/
          unsigned char _print_aux[17];             /*Conjunto de enderecos tipo char que recebe os caracteres*/
                                                    /* correspondente aos numeros apos serem convertidos*/
          char _signed;                             /*Esta variavel serve para determinar se um numero e positivo ou negativo*/
          _count = 0;                               /*Coloca variavel _cont em 0 para iniciar o primeiro passo do algoritimo*/

          /*_aux = _print * pow(10.0,_decimal);       O laco a seguir e uma implementacao que realiza uma operacao exponencial
                                                      Sem bibliotecas, menor problemas com diferentes compiladores*/
          while( _count < _decimal) {               /*Enquando _count menor que _decimal*/

				_count++;                           /*Incrementa _count*/
                _print*=10.0;                       /*Multiplica parametro _print por 10
                                                       e guarda o resultado na propria variavel _print
                                                      Se enviado _decimal = 0, _count(0) nao sera menor que _decimal(0)
                                                      Portanto nao sera entrado no laco
                                                       e a parte decimal do numero enviado nao sera impresso no display LCD
                                                      Caso seja passado algum valor para o parametro _decimal
                                                       este laco vai ser repetido _decimal vezes
                                                      Ao ir mutiplicando por 10 o valor passado para o parametro _print,
                                                       estamos deslocando os valores decimais para a parte dos inteiros
                                                      A variavel _count e incrementada ate o valor _decimal*/

          }

          _aux = (long int)_print;                  /*A parte inteira do valor e salva na variavel _aux*/

          if(_aux&(1UL<<31)) {                      /*Testa se foi enviado um numero negativo
                                                      O bit mais significativo da variavel determina o sinal;
                                                       1=negativo; 0=positivo
                                                      Se for negativo devemos ajustar este numero
                                                       e setar alguma flag que indicara que deve ser impresso um sinal '-'*/

				_aux=~_aux+1;                       /*Numeros negativos sao visto pelo processador como inversos aos positivos
                                                      Ao inverter os bits com a operacao de complemento, inverte-se o sinal
                                                      A adicao de +1 serve para adicionar o 0
                                                       que esta do lado dos numeros positivos
                                                       e nao entra no lado dos numeros negativos*/

                _signed = 1;                        /*Seta _signed para indicar que deve ser impresso o '-'*/
          }
          else _signed = 0;                         /*Se nao for um numero negativo
                                                      Limpa _signed para indicar que  nao deve ser impresso o '-'*/


          _count = 1;                               /*Seta variavel _count para a proxima parte do algoritimo
                                                      Agora _count sera usada como ponteiro
                                                      Ele indicara enderecos onde serao salvos os numeros convertidos*/

          if(_decimal) {                            /*Se foi requisitado algum numero decimal o algoritimo entra nesse laco
                                                      Ele transforma a parte decimal em caracteres*/

				while(_decimal) {                   /*Enquanto tiver numero decimal*/

				  _print_aux[_count]=(_aux%10)+'0'; /*Esta operacao retira os numeros da variavel _aux
                                                      A operacao (_aux%10) separa a unidade dos numeros
                                                      A operacao +'0' converte a unidade no seu caractere da tabela ASCII
                                                      O caracter convertido e salvo no endereco _print_aux[]
                                                       apontado por _count*/

                          _aux/=10;                 /*_aux e dividido por 10
                                                      O resultado da operacao e salva na propria variavel _aux
                                                      Isto faz com que o proximo numero
                                                      passe pelo processo de separacao e convercao*/

                          _count++;                 /* _count e incremetado para apontar para um novo endereco*/

                          _decimal--;               /*Decrementa variavel _decimal e indica que um decimal requisitado
                                                       ja foi processado e salvo como caractere*/

                  }

                  _print_aux[_count]=',';           /*Quando todos os decimais requisitados foram convertidos
                                                      Adiciona uma "virgula" para indicar a parte deimal do numero
                                                      Caso se deseje a notacao norte americana para pontos flutuantes,
                                                      basta mudar a pontuacao para '.'*/

                  _count++;                         /*Incrementa o endereco para guardar os proximos caracteres*/

          }


          do {                                      /*Executa a operacao pelo menos uma vez
                                                      para que seja impresso pelo menos um numero 0 quando enviado sozinho*/

                  _print_aux[_count]=(_aux%10)+'0'; /*Operacao de separacao e convercao da parte inteira do numero
                                                      A operacao _aux%10 separa a unidade do numero
                                                      A operacao +'0' converte a unidade para o seu caractere da tabela ASCII*/

                  _aux/=10;                         /*Divide _aux por 10 para processar o proximo numero*/
                  _count++;                         /*Incrementa ponteiro para o proximo endereco da variavel _print_aux[]*/

          } while (_aux);                           /*Repete esta operacao enquanto haver um numero
                                                       diferente de zero para ser processado*/

          if (_signed)                              /*Testa se _signed foi setado*/
          _print_aux[_count] = '-';                 /*Se sim, o numero enviado e negativo e impresso '-' no inicio do numero*/
          else                                      /*Se nao for um numero negativo*/
          _count--;                                 /*Decrementa um endereco para corrigir posicionamento da variavel*/

          while (_count) {                          /*Inicio da impressao dos caracteres convertidos
                                                       que estao organizados na variavel _print_aux[]
                                                      Os numeros foram salvos invertidos (de tras para frente),
                                                       entao deve-se ir decrementando _count para imprimir
                                                      Enquanto o apontador de enderecos _count nao zerar
                                                       significa que existem caracteres para imprimir*/
                  lcd_d_c(_print_aux[_count],dado); /*Imprime o caractere no endereco apontado por _count*/
                  _count--;                         /*Decrementa _count para pontar para o proximo caracter a ser impresso*/
          }

  }
