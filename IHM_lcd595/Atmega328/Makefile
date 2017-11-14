#Used with avr
#Dependences: avr-gcc, avr-binutils

#compiler
compiler = avr-gcc
hexgenerator = avr-objcopy

#options of code source
flags = -Os -g2 -gdwarf-2
mcu = atmega328
app	= lcd_test


all:
	$(compiler) $(flags) -mmcu=$(mcu) -c *.c
	$(compiler) $(flags) -mmcu=$(mcu) *.o -o $(app).elf
	$(hexgenerator) -O ihex $(app).elf $(app).hex
	avr-size -C  -x --mcu=$(mcu) $(app).elf
