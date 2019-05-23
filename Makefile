main_file:
	avr-gcc -std=c11 -g -Os -mmcu=atmega8 -c nowy.c
	avr-gcc -g -mmcu=atmega8 -o nowy.elf nowy.o
	avr-objcopy -j .text -j .data -O ihex nowy.elf nowy.hex
	avrdude -c usbasp -p m8 -U flash:w:nowy.hex
