#Used with SDCC
#Dependences: sdcc

#compiler
compiler = sdcc
hexgenerator = packihx

#options of code source
flags = --opt-code-size
mcu = mcs51
source = main
libary = IHM_lcd595
app	= lcd_test


all:
	$(compiler) $(flags)  -m$(mcu) -c $(source).c
	$(compiler) $(flags)  -m$(mcu) -c $(libary).c
	$(compiler) $(flags)  -m$(mcu) $(source).rel $(libary).rel -o $(app).ihx
	$(hexgenerator) $(app).ihx > $(app).hex
	size $(app).hex

clear:
	rm *.rel
	rm *.rst
	rm *.sym
	rm *.lst
	rm *.lk
	
