#Used with SDCC
#Dependences: sdcc

#compiler
compiler = sdcc
hexgenerator = packihx

#options of code source
flags = --opt-code-size
mcu = mcs51
source = main


all:
	$(compiler) $(flags)  -m$(mcu) $(source).c
	$(hexgenerator) $(source).ihx > $(source).hex
	size $(source).hex
