ifndef TTY
TTY=/dev/ttyACM0
endif

# compiler flags:
# -Os                : optimize size
# -DF_CPU=16000000UL : define F_CPU as 16Mghz
# -mmcu=atmega328p   : target MCU

main.hex : main.c
	# compile to object file (optimize for size)
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p  -o main.elf main.c twi.c
	# link as ELF binary
	#avr-gcc -mmcu=atmega328p main.o -o main.elf
	# convert ELF format to an IHEX format as used by avrdude
	avr-objcopy -O ihex -R .eeprom main.elf main.hex


burn:
	avrdude -F -V -c arduino -p ATMEGA328P -P $(TTY) -b 115200 -U flash:w:main.hex

clean :
	rm main.o twi.o main.elf main.hex
