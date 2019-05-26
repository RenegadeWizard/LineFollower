main_file:
	avr-gcc -std=c11 -g -Os -mmcu=atmega328p -c nowy.c
	avr-gcc -g -mmcu=atmega328p -o nowy.elf nowy.o
	avr-objcopy -j .text -j .data -O ihex nowy.elf nowy.hex
	avrdude -c usbasp -p m328p -U flash:w:nowy.hex
