;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.0 #9615 (MINGW32)
;--------------------------------------------------------
	.module main
	.optsdcc -mmcs51 --model-small
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _lcd_printN
	.globl _lcd_printS
	.globl _lcd_backlight
	.globl _lcd_set
	.globl _lcd_begin
	.globl __delay_us
	.globl __delay_ms
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
	.globl _count
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
_count::
	.ds 2
;--------------------------------------------------------
; overlayable items in internal ram 
;--------------------------------------------------------
	.area	OSEG    (OVR,DATA)
	.area	OSEG    (OVR,DATA)
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

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
; interrupt vector 
;--------------------------------------------------------
	.area HOME    (CODE)
__interrupt_vect:
	ljmp	__sdcc_gsinit_startup
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME    (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area GSINIT  (CODE)
	.globl __sdcc_gsinit_startup
	.globl __sdcc_program_startup
	.globl __start__stack
	.globl __mcs51_genXINIT
	.globl __mcs51_genXRAMCLEAR
	.globl __mcs51_genRAMCLEAR
;	main.c:14: int count = 0;
	clr	a
	mov	_count,a
	mov	(_count + 1),a
	.area GSFINAL (CODE)
	ljmp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME    (CODE)
	.area HOME    (CODE)
__sdcc_program_startup:
	ljmp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CSEG    (CODE)
;------------------------------------------------------------
;Allocation info for local variables in function '_delay_ms'
;------------------------------------------------------------
;ms                        Allocated to registers r6 r7 
;------------------------------------------------------------
;	delay.h:42: void _delay_ms(int ms) {
;	-----------------------------------------
;	 function _delay_ms
;	-----------------------------------------
__delay_ms:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
	mov	r6,dpl
	mov	r7,dph
;	delay.h:43: TR0 = 0;
	clr	_TR0
;	delay.h:44: TF0 = 0;
	clr	_TF0
;	delay.h:45: TMOD |= 1;
	orl	_TMOD,#0x01
;	delay.h:46: while(ms) {
00104$:
	mov	a,r6
	orl	a,r7
	jz	00107$
;	delay.h:47: TL0 = 0x18;
	mov	_TL0,#0x18
;	delay.h:48: TH0 = 0xFC;
	mov	_TH0,#0xfc
;	delay.h:49: TR0 = 1;
	setb	_TR0
;	delay.h:50: ms--;
	dec	r6
	cjne	r6,#0xff,00123$
	dec	r7
00123$:
;	delay.h:51: while(!TF0);
00101$:
	jnb	_TF0,00101$
;	delay.h:52: TR0 = 0;
	clr	_TR0
;	delay.h:53: TF0 = 0;
	clr	_TF0
	sjmp	00104$
00107$:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function '_delay_us'
;------------------------------------------------------------
;ms                        Allocated to registers r6 r7 
;------------------------------------------------------------
;	delay.h:58: void _delay_us(int ms) {
;	-----------------------------------------
;	 function _delay_us
;	-----------------------------------------
__delay_us:
	mov	r6,dpl
	mov	r7,dph
;	delay.h:59: TR0 = 0;
	clr	_TR0
;	delay.h:60: TF0 = 0;
	clr	_TF0
;	delay.h:61: TL0 = (255-(ms&0x0F));
	mov	a,#0x0f
	anl	a,r6
	mov	r4,a
	mov	r5,#0x00
	mov	a,#0xff
	clr	c
	subb	a,r4
	mov	_TL0,a
;	delay.h:62: TH0 = (255-(ms>>8));
	mov	ar6,r7
	mov	a,#0xff
	clr	c
	subb	a,r6
	mov	_TH0,a
;	delay.h:63: TR0 = 1;
	setb	_TR0
;	delay.h:64: while(!TF0);
00101$:
	jnb	_TF0,00101$
;	delay.h:65: TR0 = 0;
	clr	_TR0
;	delay.h:66: TF0 = 0;
	clr	_TF0
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'main'
;------------------------------------------------------------
;	main.c:16: int main() {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
;	main.c:18: lcd_begin();
	lcall	_lcd_begin
;	main.c:19: lcd_backlight(on);
	mov	dpl,#0x01
	lcall	_lcd_backlight
;	main.c:20: lcd_set(0,0);
	mov	_lcd_set_PARM_2,#0x00
	mov	dpl,#0x00
	lcall	_lcd_set
;	main.c:21: lcd_printS("Hello World!");
	mov	dptr,#___str_0
	mov	b,#0x80
	lcall	_lcd_printS
00102$:
;	main.c:23: lcd_set(0,1);
	mov	_lcd_set_PARM_2,#0x01
	mov	dpl,#0x00
	lcall	_lcd_set
;	main.c:24: lcd_printN(count++,0);
	mov	dpl,_count
	mov	dph,(_count + 1)
	inc	_count
	clr	a
	cjne	a,_count,00109$
	inc	(_count + 1)
00109$:
	lcall	___sint2fs
	mov	r4,dpl
	mov	r5,dph
	mov	r6,b
	mov	r7,a
	mov	_lcd_printN_PARM_2,#0x00
	mov	dpl,r4
	mov	dph,r5
	mov	b,r6
	mov	a,r7
	lcall	_lcd_printN
;	main.c:25: lcd_printS("     ");
	mov	dptr,#___str_1
	mov	b,#0x80
	lcall	_lcd_printS
;	main.c:26: P0_0 = 0;
	clr	_P0_0
;	main.c:27: _delay_ms(500);
	mov	dptr,#0x01f4
	lcall	__delay_ms
;	main.c:28: P0_0 = 1;
	setb	_P0_0
;	main.c:29: _delay_ms(500);
	mov	dptr,#0x01f4
	lcall	__delay_ms
;	main.c:30: lcd_set(0,1);
	mov	_lcd_set_PARM_2,#0x01
	mov	dpl,#0x00
	lcall	_lcd_set
	sjmp	00102$
	.area CSEG    (CODE)
	.area CONST   (CODE)
___str_0:
	.ascii "Hello World!"
	.db 0x00
___str_1:
	.ascii "     "
	.db 0x00
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
