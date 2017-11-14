;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.0 #9615 (MINGW32)
;--------------------------------------------------------
	.module IHM_lcd595
	.optsdcc -mmcs51 --model-small
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _CY
	.globl _AC
	.globl _F0
	.globl _RS1
	.globl _RS0
	.globl _OV
	.globl _FL
	.globl _P
	.globl _PS
	.globl _PT1
	.globl _PX1
	.globl _PT0
	.globl _PX0
	.globl _RD
	.globl _WR
	.globl _T1
	.globl _T0
	.globl _INT1
	.globl _INT0
	.globl _TXD
	.globl _RXD
	.globl _P3_7
	.globl _P3_6
	.globl _P3_5
	.globl _P3_4
	.globl _P3_3
	.globl _P3_2
	.globl _P3_1
	.globl _P3_0
	.globl _EA
	.globl _ES
	.globl _ET1
	.globl _EX1
	.globl _ET0
	.globl _EX0
	.globl _P2_7
	.globl _P2_6
	.globl _P2_5
	.globl _P2_4
	.globl _P2_3
	.globl _P2_2
	.globl _P2_1
	.globl _P2_0
	.globl _SM0
	.globl _SM1
	.globl _SM2
	.globl _REN
	.globl _TB8
	.globl _RB8
	.globl _TI
	.globl _RI
	.globl _P1_7
	.globl _P1_6
	.globl _P1_5
	.globl _P1_4
	.globl _P1_3
	.globl _P1_2
	.globl _P1_1
	.globl _P1_0
	.globl _TF1
	.globl _TR1
	.globl _TF0
	.globl _TR0
	.globl _IE1
	.globl _IT1
	.globl _IE0
	.globl _IT0
	.globl _P0_7
	.globl _P0_6
	.globl _P0_5
	.globl _P0_4
	.globl _P0_3
	.globl _P0_2
	.globl _P0_1
	.globl _P0_0
	.globl _B
	.globl _A
	.globl _ACC
	.globl _PSW
	.globl _IP
	.globl _P3
	.globl _IE
	.globl _P2
	.globl _SBUF
	.globl _SCON
	.globl _P1
	.globl _TH1
	.globl _TH0
	.globl _TL1
	.globl _TL0
	.globl _TMOD
	.globl _TCON
	.globl _PCON
	.globl _DPH
	.globl _DPL
	.globl _SP
	.globl _P0
	.globl _lcd_printN_PARM_2
	.globl _lcd_set_PARM_2
	.globl ____background
	.globl _lcd_begin
	.globl _lcd_clear
	.globl _lcd_home
	.globl _lcd_cursor
	.globl _lcd_display
	.globl _lcd_set
	.globl _lcd_backlight
	.globl _lcd_printC
	.globl _lcd_printS
	.globl _lcd_printN
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
	.area RSEG    (ABS,DATA)
	.org 0x0000
_P0	=	0x0080
_SP	=	0x0081
_DPL	=	0x0082
_DPH	=	0x0083
_PCON	=	0x0087
_TCON	=	0x0088
_TMOD	=	0x0089
_TL0	=	0x008a
_TL1	=	0x008b
_TH0	=	0x008c
_TH1	=	0x008d
_P1	=	0x0090
_SCON	=	0x0098
_SBUF	=	0x0099
_P2	=	0x00a0
_IE	=	0x00a8
_P3	=	0x00b0
_IP	=	0x00b8
_PSW	=	0x00d0
_ACC	=	0x00e0
_A	=	0x00e0
_B	=	0x00f0
;--------------------------------------------------------
; special function bits
;--------------------------------------------------------
	.area RSEG    (ABS,DATA)
	.org 0x0000
_P0_0	=	0x0080
_P0_1	=	0x0081
_P0_2	=	0x0082
_P0_3	=	0x0083
_P0_4	=	0x0084
_P0_5	=	0x0085
_P0_6	=	0x0086
_P0_7	=	0x0087
_IT0	=	0x0088
_IE0	=	0x0089
_IT1	=	0x008a
_IE1	=	0x008b
_TR0	=	0x008c
_TF0	=	0x008d
_TR1	=	0x008e
_TF1	=	0x008f
_P1_0	=	0x0090
_P1_1	=	0x0091
_P1_2	=	0x0092
_P1_3	=	0x0093
_P1_4	=	0x0094
_P1_5	=	0x0095
_P1_6	=	0x0096
_P1_7	=	0x0097
_RI	=	0x0098
_TI	=	0x0099
_RB8	=	0x009a
_TB8	=	0x009b
_REN	=	0x009c
_SM2	=	0x009d
_SM1	=	0x009e
_SM0	=	0x009f
_P2_0	=	0x00a0
_P2_1	=	0x00a1
_P2_2	=	0x00a2
_P2_3	=	0x00a3
_P2_4	=	0x00a4
_P2_5	=	0x00a5
_P2_6	=	0x00a6
_P2_7	=	0x00a7
_EX0	=	0x00a8
_ET0	=	0x00a9
_EX1	=	0x00aa
_ET1	=	0x00ab
_ES	=	0x00ac
_EA	=	0x00af
_P3_0	=	0x00b0
_P3_1	=	0x00b1
_P3_2	=	0x00b2
_P3_3	=	0x00b3
_P3_4	=	0x00b4
_P3_5	=	0x00b5
_P3_6	=	0x00b6
_P3_7	=	0x00b7
_RXD	=	0x00b0
_TXD	=	0x00b1
_INT0	=	0x00b2
_INT1	=	0x00b3
_T0	=	0x00b4
_T1	=	0x00b5
_WR	=	0x00b6
_RD	=	0x00b7
_PX0	=	0x00b8
_PT0	=	0x00b9
_PX1	=	0x00ba
_PT1	=	0x00bb
_PS	=	0x00bc
_P	=	0x00d0
_FL	=	0x00d1
_OV	=	0x00d2
_RS0	=	0x00d3
_RS1	=	0x00d4
_F0	=	0x00d5
_AC	=	0x00d6
_CY	=	0x00d7
;--------------------------------------------------------
; overlayable register banks
;--------------------------------------------------------
	.area REG_BANK_0	(REL,OVR,DATA)
	.ds 8
;--------------------------------------------------------
; internal ram data
;--------------------------------------------------------
	.area DSEG    (DATA)
____background::
	.ds 1
_lcd_cmd_PARM_2:
	.ds 1
_lcd_set_PARM_2:
	.ds 1
_lcd_set_count_4_36:
	.ds 2
_lcd_printN_PARM_2:
	.ds 1
_lcd_printN__count_1_45:
	.ds 1
_lcd_printN__aux_1_45:
	.ds 4
_lcd_printN__print_aux_1_45:
	.ds 17
_lcd_printN__signed_1_45:
	.ds 1
;--------------------------------------------------------
; overlayable items in internal ram 
;--------------------------------------------------------
	.area	OSEG    (OVR,DATA)
_lcd_delay40us_count_1_12:
	.ds 2
	.area	OSEG    (OVR,DATA)
_lcd_delay4ms_count_1_13:
	.ds 2
	.area	OSEG    (OVR,DATA)
_lcd_send_count_4_19:
	.ds 2
;--------------------------------------------------------
; indirectly addressable internal ram data
;--------------------------------------------------------
	.area ISEG    (DATA)
;--------------------------------------------------------
; absolute internal ram data
;--------------------------------------------------------
	.area IABS    (ABS,DATA)
	.area IABS    (ABS,DATA)
;--------------------------------------------------------
; bit data
;--------------------------------------------------------
	.area BSEG    (BIT)
;--------------------------------------------------------
; paged external ram data
;--------------------------------------------------------
	.area PSEG    (PAG,XDATA)
;--------------------------------------------------------
; external ram data
;--------------------------------------------------------
	.area XSEG    (XDATA)
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area XABS    (ABS,XDATA)
;--------------------------------------------------------
; external initialized ram data
;--------------------------------------------------------
	.area XISEG   (XDATA)
	.area HOME    (CODE)
	.area GSINIT0 (CODE)
	.area GSINIT1 (CODE)
	.area GSINIT2 (CODE)
	.area GSINIT3 (CODE)
	.area GSINIT4 (CODE)
	.area GSINIT5 (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area CSEG    (CODE)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME    (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area GSINIT  (CODE)
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME    (CODE)
	.area HOME    (CODE)
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CSEG    (CODE)
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_delay40us'
;------------------------------------------------------------
;count                     Allocated with name '_lcd_delay40us_count_1_12'
;------------------------------------------------------------
;	IHM_lcd595.c:13: inline static void lcd_delay40us() {
;	-----------------------------------------
;	 function lcd_delay40us
;	-----------------------------------------
_lcd_delay40us:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
;	IHM_lcd595.c:14: volatile unsigned int count = sendTick;
	mov	_lcd_delay40us_count_1_12,#0x02
	mov	(_lcd_delay40us_count_1_12 + 1),#0x00
;	IHM_lcd595.c:15: while(count--);
00101$:
	mov	r6,_lcd_delay40us_count_1_12
	mov	r7,(_lcd_delay40us_count_1_12 + 1)
	dec	_lcd_delay40us_count_1_12
	mov	a,#0xff
	cjne	a,_lcd_delay40us_count_1_12,00109$
	dec	(_lcd_delay40us_count_1_12 + 1)
00109$:
	mov	a,r6
	orl	a,r7
	jnz	00101$
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_delay4ms'
;------------------------------------------------------------
;count                     Allocated with name '_lcd_delay4ms_count_1_13'
;------------------------------------------------------------
;	IHM_lcd595.c:18: static void lcd_delay4ms() {
;	-----------------------------------------
;	 function lcd_delay4ms
;	-----------------------------------------
_lcd_delay4ms:
;	IHM_lcd595.c:19: volatile unsigned int count = initTick;
	mov	_lcd_delay4ms_count_1_13,#0xfa
	mov	(_lcd_delay4ms_count_1_13 + 1),#0x00
;	IHM_lcd595.c:20: while(count--);
00101$:
	mov	r6,_lcd_delay4ms_count_1_13
	mov	r7,(_lcd_delay4ms_count_1_13 + 1)
	dec	_lcd_delay4ms_count_1_13
	mov	a,#0xff
	cjne	a,_lcd_delay4ms_count_1_13,00109$
	dec	(_lcd_delay4ms_count_1_13 + 1)
00109$:
	mov	a,r6
	orl	a,r7
	jnz	00101$
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_send'
;------------------------------------------------------------
;_dado                     Allocated to registers r7 
;_aux                      Allocated to registers r6 
;count                     Allocated with name '_lcd_send_count_4_19'
;------------------------------------------------------------
;	IHM_lcd595.c:25: static void lcd_send(unsigned char _dado) {
;	-----------------------------------------
;	 function lcd_send
;	-----------------------------------------
_lcd_send:
	mov	r7,dpl
;	IHM_lcd595.c:29: for(_aux=0; _aux<6; _aux++){          /*Envia dado bit a bit para display ate o sexto bit*/
	mov	r6,#0x00
00109$:
;	IHM_lcd595.c:31: if(_dado&0x01) _data;               /*Verifica se dado a ser enviado e 1 comparando com mascara 0b00000001*/
	mov	a,r7
	jnb	acc.0,00102$
	setb	_P3_1
	sjmp	00103$
00102$:
;	IHM_lcd595.c:38: else  _Ndata;                       /*Se nao, limpa o pino escolhido como _data e envia 0 para deslocador*/
	clr	_P3_1
00103$:
;	IHM_lcd595.c:41: _clock;                             /*Borda de subida para deslocar dado enviado no deslocador de bit 74LS595*/
	setb	_P3_0
;	IHM_lcd595.c:42: _Nclock;                            /*Borda de descida para deslocar dado enviado no deslocador de bit 74LS595*/
	clr	_P3_0
;	IHM_lcd595.c:44: _dado = _dado>>1;                   /*Desloca dado uma casa para a direita para enviar o proximo bit*/
	mov	a,r7
	clr	c
	rrc	a
	mov	r7,a
;	IHM_lcd595.c:29: for(_aux=0; _aux<6; _aux++){          /*Envia dado bit a bit para display ate o sexto bit*/
	inc	r6
	cjne	r6,#0x06,00127$
00127$:
	jc	00109$
;	IHM_lcd595.c:48: _enable;							  /*Borda de subida para o sinal de enable*/
	setb	_P3_2
;	IHM_lcd595.c:49: _Nenable;							  /*Borda de descida para o sinal de enable*/
	clr	_P3_2
;	IHM_lcd595.c:14: volatile unsigned int count = sendTick;
	mov	_lcd_send_count_4_19,#0x02
	mov	(_lcd_send_count_4_19 + 1),#0x00
;	IHM_lcd595.c:15: while(count--);
00105$:
	mov	r6,_lcd_send_count_4_19
	mov	r7,(_lcd_send_count_4_19 + 1)
	dec	_lcd_send_count_4_19
	mov	a,#0xff
	cjne	a,_lcd_send_count_4_19,00129$
	dec	(_lcd_send_count_4_19 + 1)
00129$:
	mov	a,r6
	orl	a,r7
	jnz	00105$
;	IHM_lcd595.c:50: lcd_delay40us();                  	  /*Aguarda tempo para LCD aceitar instrucao*/
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_cmd'
;------------------------------------------------------------
;_comando                  Allocated with name '_lcd_cmd_PARM_2'
;_dado                     Allocated to registers r7 
;_enviar                   Allocated to registers r6 
;------------------------------------------------------------
;	IHM_lcd595.c:55: static void lcd_cmd(unsigned char _dado, unsigned char _comando) {
;	-----------------------------------------
;	 function lcd_cmd
;	-----------------------------------------
_lcd_cmd:
;	IHM_lcd595.c:62: _enviar = _dado>>4;                   /*Salva na variavel o nibble mais significativo do dado
	mov	a,dpl
	mov	r7,a
	swap	a
	anl	a,#0x0f
	mov	r6,a
;	IHM_lcd595.c:67: if (_comando) _enviar &= ~(1<<4);     /*Se parametro comando for 1, altera o 5 bit para 0*/
	mov	a,_lcd_cmd_PARM_2
	jz	00102$
	anl	ar6,#0xef
	sjmp	00103$
00102$:
;	IHM_lcd595.c:68: else _enviar|=(1<<4);                 /*Se nao, se parametro comando for 0, altera o 5 bit para 1*/
	orl	ar6,#0x10
00103$:
;	IHM_lcd595.c:71: if (___background) _enviar |= (1<<5); /*Se background do display setado como ligado, altera o 6 bit para 1*/
	mov	a,____background
	jz	00105$
	orl	ar6,#0x20
	sjmp	00106$
00105$:
;	IHM_lcd595.c:72: else _enviar &= ~(1<<5);              /*Senao, altera o 6 bit para 0*/
	anl	ar6,#0xdf
00106$:
;	IHM_lcd595.c:75: lcd_send(_enviar);                    /*_enviado = 0b00BR DDDD; envia os 4 bits mais significativos para o display
	mov	dpl,r6
	push	ar7
	lcall	_lcd_send
	pop	ar7
;	IHM_lcd595.c:83: if (_comando) _enviar &= ~(1<<4);     /*Se parametro comando for 1, altera o 5 bit para 0*/
	mov	a,_lcd_cmd_PARM_2
	jz	00108$
	mov	a,#0xef
	anl	a,r7
	mov	r6,a
	sjmp	00109$
00108$:
;	IHM_lcd595.c:84: else _enviar|=(1<<4);                 /*Se nao, se parametro comando for 0, altera o 5 bit para 1*/
	mov	a,#0x10
	orl	a,r7
	mov	r6,a
00109$:
;	IHM_lcd595.c:87: if (___background) _enviar |= (1<<5); /*Se background do display setado como ligado altera o 6 bit para 1*/
	mov	a,____background
	jz	00111$
	orl	ar6,#0x20
	sjmp	00112$
00111$:
;	IHM_lcd595.c:88: else _enviar &= ~(1<<5);              /*Senao, altera o 6 bit para 0
	anl	ar6,#0xdf
00112$:
;	IHM_lcd595.c:91: lcd_send(_enviar);                    /*_enviado = 0bxxBR dddd; envia os 4 bits menos significativos para o display
	mov	dpl,r6
	ljmp	_lcd_send
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_begin'
;------------------------------------------------------------
;	IHM_lcd595.c:100: void lcd_begin(void) {
;	-----------------------------------------
;	 function lcd_begin
;	-----------------------------------------
_lcd_begin:
;	IHM_lcd595.c:102: _Nclock;
	clr	_P3_0
;	IHM_lcd595.c:103: _Ndata;
	clr	_P3_1
;	IHM_lcd595.c:104: _Nenable;
	clr	_P3_2
;	IHM_lcd595.c:112: lcd_cmd(0x30,comando);
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x30
	lcall	_lcd_cmd
;	IHM_lcd595.c:113: lcd_delay4ms();
	lcall	_lcd_delay4ms
;	IHM_lcd595.c:114: lcd_cmd(0x30,comando);
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x30
	lcall	_lcd_cmd
;	IHM_lcd595.c:115: lcd_delay4ms();
	lcall	_lcd_delay4ms
;	IHM_lcd595.c:116: lcd_cmd(0x30,comando);				  /*Tres tentativas para resolver bugs*/
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x30
	lcall	_lcd_cmd
;	IHM_lcd595.c:117: lcd_delay4ms();
	lcall	_lcd_delay4ms
;	IHM_lcd595.c:119: lcd_cmd(0x02,comando);                /*Envia comando para colocar lcd em modo de 4bits*/
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x02
	lcall	_lcd_cmd
;	IHM_lcd595.c:120: lcd_delay4ms();                		  /*Aguarda tempo de 4.5ms para inicializacao
	lcall	_lcd_delay4ms
;	IHM_lcd595.c:125: lcd_cmd(0x28,comando);                /*Envia comando para configurar lcd como 2 linhas e matriz 5x8*/
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x28
	lcall	_lcd_cmd
;	IHM_lcd595.c:126: lcd_delay4ms();                		  /*Aguarda tempo de 4.5ms para inicializacao
	lcall	_lcd_delay4ms
;	IHM_lcd595.c:131: lcd_cursor(desligado);           	  /*Envia comando para configurar cursor desligado como padrao
	mov	dpl,#0x0c
	lcall	_lcd_cursor
;	IHM_lcd595.c:135: lcd_clear();				 		  /*Garante que display esteja limpo a inicializar*/
	lcall	_lcd_clear
;	IHM_lcd595.c:136: lcd_set(0,0);						  /*Posiciona cursor no inicio*/
	mov	_lcd_set_PARM_2,#0x00
	mov	dpl,#0x00
	ljmp	_lcd_set
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_clear'
;------------------------------------------------------------
;	IHM_lcd595.c:141: void lcd_clear(void) {
;	-----------------------------------------
;	 function lcd_clear
;	-----------------------------------------
_lcd_clear:
;	IHM_lcd595.c:143: lcd_cmd(0x01,comando);                /*Envia comando para limpar o display lcd*/
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x01
	lcall	_lcd_cmd
;	IHM_lcd595.c:144: lcd_delay4ms();                       /*Aguarda tempo de 2ms para a instrucao ser executada
	ljmp	_lcd_delay4ms
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_home'
;------------------------------------------------------------
;	IHM_lcd595.c:152: void lcd_home(void) {
;	-----------------------------------------
;	 function lcd_home
;	-----------------------------------------
_lcd_home:
;	IHM_lcd595.c:154: lcd_cmd(0x02,comando);                /*Envia comando para retornar o cursor do display para o inicio*/
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x02
	lcall	_lcd_cmd
;	IHM_lcd595.c:155: lcd_delay4ms();                       /*Aguarda tempo de 2ms para a instrucao ser executada
	ljmp	_lcd_delay4ms
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_cursor'
;------------------------------------------------------------
;_modo                     Allocated to registers 
;------------------------------------------------------------
;	IHM_lcd595.c:163: void lcd_cursor(unsigned char _modo) {  /*Parametro _modo e a forma com que o cursor do display ira se comportar
;	-----------------------------------------
;	 function lcd_cursor
;	-----------------------------------------
_lcd_cursor:
;	IHM_lcd595.c:170: lcd_cmd(_modo,comando);               /*Envia comando para mudar o modo do display*/
	mov	_lcd_cmd_PARM_2,#0x01
	ljmp	_lcd_cmd
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_display'
;------------------------------------------------------------
;mode                      Allocated to registers r7 
;------------------------------------------------------------
;	IHM_lcd595.c:176: void lcd_display(char mode) {
;	-----------------------------------------
;	 function lcd_display
;	-----------------------------------------
_lcd_display:
;	IHM_lcd595.c:177: if(mode)
	mov	a,dpl
	mov	r7,a
	jz	00102$
;	IHM_lcd595.c:178: lcd_cmd(0x08,comando);            /*Envia comando para desligar o display
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x08
	ljmp	_lcd_cmd
00102$:
;	IHM_lcd595.c:182: else lcd_cmd(0x0C,comando);           /*Envia comando para ligar o display
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x0c
	ljmp	_lcd_cmd
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_set'
;------------------------------------------------------------
;_linha                    Allocated with name '_lcd_set_PARM_2'
;_coluna                   Allocated to registers r7 
;_aux                      Allocated to registers r6 
;count                     Allocated with name '_lcd_set_count_4_36'
;------------------------------------------------------------
;	IHM_lcd595.c:190: void lcd_set(unsigned char _coluna, unsigned char _linha) {
;	-----------------------------------------
;	 function lcd_set
;	-----------------------------------------
_lcd_set:
	mov	r7,dpl
;	IHM_lcd595.c:203: if(_linha) _aux = 0xC0;               /*Se parametro _linha for 1 envia o endereco da segunda linha do display
	mov	a,_lcd_set_PARM_2
	jz	00102$
	mov	r6,#0xc0
	sjmp	00103$
00102$:
;	IHM_lcd595.c:205: else _aux = 0x80;                     /*Se nao, quando envia 0 para parametro
	mov	r6,#0x80
00103$:
;	IHM_lcd595.c:208: _aux |= _coluna;                      /*Realiza uma operacao | (OR) com o nibble de coluna
	mov	a,r7
	orl	ar6,a
;	IHM_lcd595.c:215: lcd_cmd(_aux,comando);                /*Envia o comando para para posicionar o cursor do display no endereco*/
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,r6
	lcall	_lcd_cmd
;	IHM_lcd595.c:14: volatile unsigned int count = sendTick;
	mov	_lcd_set_count_4_36,#0x02
	mov	(_lcd_set_count_4_36 + 1),#0x00
;	IHM_lcd595.c:15: while(count--);
00104$:
	mov	r6,_lcd_set_count_4_36
	mov	r7,(_lcd_set_count_4_36 + 1)
	dec	_lcd_set_count_4_36
	mov	a,#0xff
	cjne	a,_lcd_set_count_4_36,00117$
	dec	(_lcd_set_count_4_36 + 1)
00117$:
	mov	a,r6
	orl	a,r7
	jnz	00104$
;	IHM_lcd595.c:216: lcd_delay40us();                	  /*Aguarda o display realizar o posicionamento
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_backlight'
;------------------------------------------------------------
;_on_off                   Allocated to registers 
;------------------------------------------------------------
;	IHM_lcd595.c:222: void lcd_backlight(unsigned char _on_off) {
;	-----------------------------------------
;	 function lcd_backlight
;	-----------------------------------------
_lcd_backlight:
	mov	____background,dpl
;	IHM_lcd595.c:229: lcd_cmd(0x00,comando);                /*Envia para o display um comando NOP (0x00)
	mov	_lcd_cmd_PARM_2,#0x01
	mov	dpl,#0x00
	ljmp	_lcd_cmd
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_printC'
;------------------------------------------------------------
;_print                    Allocated to registers 
;------------------------------------------------------------
;	IHM_lcd595.c:240: void lcd_printC(unsigned char _print) {
;	-----------------------------------------
;	 function lcd_printC
;	-----------------------------------------
_lcd_printC:
;	IHM_lcd595.c:246: lcd_cmd(_print,dado);                 /*Envia o parametro _print para o display como um dado*/
	mov	_lcd_cmd_PARM_2,#0x00
	ljmp	_lcd_cmd
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_printS'
;------------------------------------------------------------
;_print                    Allocated to registers 
;------------------------------------------------------------
;	IHM_lcd595.c:251: void lcd_printS(const char *_print) {
;	-----------------------------------------
;	 function lcd_printS
;	-----------------------------------------
_lcd_printS:
	mov	r5,dpl
	mov	r6,dph
	mov	r7,b
;	IHM_lcd595.c:277: while(*_print!='\0') {                /*Enquanto o programa nao encontra um caracter nulo(0),
00101$:
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	lcall	__gptrget
	mov	r4,a
	jz	00104$
;	IHM_lcd595.c:279: lcd_cmd(*_print,dado);              /*Envia para o display LCD o conteudo do endereco apontado pelo ponteiro*/
	mov	_lcd_cmd_PARM_2,#0x00
	mov	dpl,r4
	push	ar7
	push	ar6
	push	ar5
	lcall	_lcd_cmd
	pop	ar5
	pop	ar6
	pop	ar7
;	IHM_lcd595.c:280: _print++;                           /*Avanca o ponteiro para a proxima posicao da string*/
	inc	r5
	cjne	r5,#0x00,00101$
	inc	r6
	sjmp	00101$
00104$:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'lcd_printN'
;------------------------------------------------------------
;_decimal                  Allocated with name '_lcd_printN_PARM_2'
;_print                    Allocated to registers r4 r5 r6 r7 
;_count                    Allocated with name '_lcd_printN__count_1_45'
;_aux                      Allocated with name '_lcd_printN__aux_1_45'
;_print_aux                Allocated with name '_lcd_printN__print_aux_1_45'
;_signed                   Allocated with name '_lcd_printN__signed_1_45'
;------------------------------------------------------------
;	IHM_lcd595.c:293: void lcd_printN(float _print, unsigned char _decimal) {
;	-----------------------------------------
;	 function lcd_printN
;	-----------------------------------------
_lcd_printN:
	mov	r4,dpl
	mov	r5,dph
	mov	r6,b
	mov	r7,a
;	IHM_lcd595.c:310: while( _count < _decimal) {               /*Enquando _count menor que _decimal*/
	mov	r3,#0x00
00101$:
	clr	c
	mov	a,r3
	subb	a,_lcd_printN_PARM_2
	jnc	00103$
;	IHM_lcd595.c:312: _count++;                           /*Incrementa _count*/
	inc	r3
;	IHM_lcd595.c:313: _print*=10.0;                       /*Multiplica parametro _print por 10
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	mov	dptr,#0x0000
	mov	b,#0x20
	mov	a,#0x41
	lcall	___fsmul
	mov	r4,dpl
	mov	r5,dph
	mov	r6,b
	mov	r7,a
	mov	a,sp
	add	a,#0xfc
	mov	sp,a
	pop	ar3
	sjmp	00101$
00103$:
;	IHM_lcd595.c:326: _aux = (long int)_print;                  /*A parte inteira do valor e salva na variavel _aux*/
	mov	dpl,r4
	mov	dph,r5
	mov	b,r6
	mov	a,r7
	lcall	___fs2slong
	mov	_lcd_printN__aux_1_45,dpl
	mov	(_lcd_printN__aux_1_45 + 1),dph
	mov	(_lcd_printN__aux_1_45 + 2),b
	mov	(_lcd_printN__aux_1_45 + 3),a
;	IHM_lcd595.c:328: if(_aux&(1UL<<31)) {                      /*Testa se foi enviado um numero negativo
	mov	a,(_lcd_printN__aux_1_45 + 3)
	jnb	acc.7,00105$
;	IHM_lcd595.c:334: _aux=~_aux+1;                       /*Numeros negativos sao visto pelo processador como inversos aos positivos
	mov	a,_lcd_printN__aux_1_45
	cpl	a
	mov	r4,a
	mov	a,(_lcd_printN__aux_1_45 + 1)
	cpl	a
	mov	r5,a
	mov	a,(_lcd_printN__aux_1_45 + 2)
	cpl	a
	mov	r6,a
	mov	a,(_lcd_printN__aux_1_45 + 3)
	cpl	a
	mov	r7,a
	mov	a,#0x01
	add	a,r4
	mov	_lcd_printN__aux_1_45,a
	clr	a
	addc	a,r5
	mov	(_lcd_printN__aux_1_45 + 1),a
	clr	a
	addc	a,r6
	mov	(_lcd_printN__aux_1_45 + 2),a
	clr	a
	addc	a,r7
	mov	(_lcd_printN__aux_1_45 + 3),a
;	IHM_lcd595.c:340: _signed = 1;                        /*Seta _signed para indicar que deve ser impresso o '-'*/
	mov	_lcd_printN__signed_1_45,#0x01
	sjmp	00106$
00105$:
;	IHM_lcd595.c:342: else _signed = 0;                         /*Se nao for um numero negativo
	mov	_lcd_printN__signed_1_45,#0x00
00106$:
;	IHM_lcd595.c:346: _count = 1;                               /*Seta variavel _count para a proxima parte do algoritimo
	mov	r6,#0x01
;	IHM_lcd595.c:350: if(_decimal) {                            /*Se foi requisitado algum numero decimal o algoritimo entra nesse laco
	mov	a,_lcd_printN_PARM_2
	jz	00128$
;	IHM_lcd595.c:353: while(_decimal) {                   /*Enquanto tiver numero decimal*/
	mov	_lcd_printN__count_1_45,#0x01
	mov	r4,_lcd_printN_PARM_2
00107$:
	mov	a,r4
	jz	00109$
;	IHM_lcd595.c:355: _print_aux[_count]=(_aux%10)+'0'; /*Esta operacao retira os numeros da variavel _aux
	mov	a,_lcd_printN__count_1_45
	add	a,#_lcd_printN__print_aux_1_45
	mov	r1,a
	mov	__modslong_PARM_2,#0x0a
	clr	a
	mov	(__modslong_PARM_2 + 1),a
	mov	(__modslong_PARM_2 + 2),a
	mov	(__modslong_PARM_2 + 3),a
	mov	dpl,_lcd_printN__aux_1_45
	mov	dph,(_lcd_printN__aux_1_45 + 1)
	mov	b,(_lcd_printN__aux_1_45 + 2)
	mov	a,(_lcd_printN__aux_1_45 + 3)
	push	ar4
	push	ar1
	lcall	__modslong
	mov	r2,dpl
	pop	ar1
	mov	a,#0x30
	add	a,r2
	mov	@r1,a
;	IHM_lcd595.c:361: _aux/=10;                 /*_aux e dividido por 10
	mov	__divslong_PARM_2,#0x0a
	clr	a
	mov	(__divslong_PARM_2 + 1),a
	mov	(__divslong_PARM_2 + 2),a
	mov	(__divslong_PARM_2 + 3),a
	mov	dpl,_lcd_printN__aux_1_45
	mov	dph,(_lcd_printN__aux_1_45 + 1)
	mov	b,(_lcd_printN__aux_1_45 + 2)
	mov	a,(_lcd_printN__aux_1_45 + 3)
	lcall	__divslong
	mov	_lcd_printN__aux_1_45,dpl
	mov	(_lcd_printN__aux_1_45 + 1),dph
	mov	(_lcd_printN__aux_1_45 + 2),b
	mov	(_lcd_printN__aux_1_45 + 3),a
	pop	ar4
;	IHM_lcd595.c:366: _count++;                 /* _count e incremetado para apontar para um novo endereco*/
	inc	_lcd_printN__count_1_45
;	IHM_lcd595.c:368: _decimal--;               /*Decrementa variavel _decimal e indica que um decimal requisitado
	dec	r4
	sjmp	00107$
00109$:
;	IHM_lcd595.c:373: _print_aux[_count]=',';           /*Quando todos os decimais requisitados foram convertidos
	mov	a,_lcd_printN__count_1_45
	add	a,#_lcd_printN__print_aux_1_45
	mov	r0,a
	mov	@r0,#0x2c
;	IHM_lcd595.c:378: _count++;                         /*Incrementa o endereco para guardar os proximos caracteres*/
	mov	a,_lcd_printN__count_1_45
	inc	a
	mov	r6,a
;	IHM_lcd595.c:383: do {                                      /*Executa a operacao pelo menos uma vez
00128$:
	mov	ar7,r6
00112$:
;	IHM_lcd595.c:386: _print_aux[_count]=(_aux%10)+'0'; /*Operacao de separacao e convercao da parte inteira do numero
	mov	a,r7
	add	a,#_lcd_printN__print_aux_1_45
	mov	r1,a
	mov	__modslong_PARM_2,#0x0a
	clr	a
	mov	(__modslong_PARM_2 + 1),a
	mov	(__modslong_PARM_2 + 2),a
	mov	(__modslong_PARM_2 + 3),a
	mov	dpl,_lcd_printN__aux_1_45
	mov	dph,(_lcd_printN__aux_1_45 + 1)
	mov	b,(_lcd_printN__aux_1_45 + 2)
	mov	a,(_lcd_printN__aux_1_45 + 3)
	push	ar7
	push	ar1
	lcall	__modslong
	mov	r2,dpl
	pop	ar1
	mov	a,#0x30
	add	a,r2
	mov	@r1,a
;	IHM_lcd595.c:390: _aux/=10;                         /*Divide _aux por 10 para processar o proximo numero*/
	mov	__divslong_PARM_2,#0x0a
	clr	a
	mov	(__divslong_PARM_2 + 1),a
	mov	(__divslong_PARM_2 + 2),a
	mov	(__divslong_PARM_2 + 3),a
	mov	dpl,_lcd_printN__aux_1_45
	mov	dph,(_lcd_printN__aux_1_45 + 1)
	mov	b,(_lcd_printN__aux_1_45 + 2)
	mov	a,(_lcd_printN__aux_1_45 + 3)
	lcall	__divslong
	mov	_lcd_printN__aux_1_45,dpl
	mov	(_lcd_printN__aux_1_45 + 1),dph
	mov	(_lcd_printN__aux_1_45 + 2),b
	mov	(_lcd_printN__aux_1_45 + 3),a
	pop	ar7
;	IHM_lcd595.c:391: _count++;                         /*Incrementa ponteiro para o proximo endereco da variavel _print_aux[]*/
	inc	r7
;	IHM_lcd595.c:393: } while (_aux);                           /*Repete esta operacao enquanto haver um numero
	mov	a,_lcd_printN__aux_1_45
	orl	a,(_lcd_printN__aux_1_45 + 1)
	orl	a,(_lcd_printN__aux_1_45 + 2)
	orl	a,(_lcd_printN__aux_1_45 + 3)
	jnz	00112$
;	IHM_lcd595.c:396: if (_signed)                              /*Testa se _signed foi setado*/
	mov	ar6,r7
	mov	a,_lcd_printN__signed_1_45
	jz	00116$
;	IHM_lcd595.c:397: _print_aux[_count] = '-';                 /*Se sim, o numero enviado e negativo e impresso '-' no inicio do numero*/
	mov	a,r7
	add	a,#_lcd_printN__print_aux_1_45
	mov	r0,a
	mov	@r0,#0x2d
	sjmp	00132$
00116$:
;	IHM_lcd595.c:399: _count--;                                 /*Decrementa um endereco para corrigir posicionamento da variavel*/
	mov	a,r7
	dec	a
	mov	r6,a
;	IHM_lcd595.c:401: while (_count) {                          /*Inicio da impressao dos caracteres convertidos
00132$:
	mov	ar7,r6
00118$:
	mov	a,r7
	jz	00121$
;	IHM_lcd595.c:407: lcd_cmd(_print_aux[_count],dado); /*Imprime o caractere no endereco apontado por _count*/
	mov	a,r7
	add	a,#_lcd_printN__print_aux_1_45
	mov	r1,a
	mov	dpl,@r1
	mov	_lcd_cmd_PARM_2,#0x00
	push	ar7
	lcall	_lcd_cmd
	pop	ar7
;	IHM_lcd595.c:408: _count--;                         /*Decrementa _count para pontar para o proximo caracter a ser impresso*/
	dec	r7
	sjmp	00118$
00121$:
	ret
	.area CSEG    (CODE)
	.area CONST   (CODE)
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
