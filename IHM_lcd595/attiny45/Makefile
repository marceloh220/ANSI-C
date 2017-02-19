#Used with avr
#Dependences: avr-gcc, avr-binutils

#compiler
compiler = avr-gcc
hexgenerator = avr-objcopy

#options of code source
flags = -Os -g2 -gdwarf-2 -Wall -v
mcu = attiny45
clock = 8000000UL
source = main


all:
	$(compiler) $(flags) -mmcu=$(mcu) -DF_CPU=$(clock) $(source).c -o $(source).elf
	$(hexgenerator) -O ihex $(source).elf $(source).hex
	avr-size -C  -x --mcu=$(mcu) $(source).elf
