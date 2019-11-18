ifndef TTY
TTY=/dev/ttyACM0
endif

MMCU=atmega328
#MMCU=atmega2560

ifeq ($(MMCU),atmega328)
PROG_MMCU=atmega328p
PROG_DEV=arduino
AVRD_CONF=
DISABLE_AUTO_ERASE=
SKIP_CHECKS=-FV
endif

ifeq ($(MMCU),atmega2560)
PROG_MMCU=$(MMCU)
PROG_DEV=wiring
AVRD_CONF=-C/etc/avrdude/avrdude.conf
DISABLE_AUTO_ERASE=-D
SKIP_CHECKS=-FV
endif



# compiler flags:
# -Os                : optimize size
# -DF_CPU=16000000UL : define F_CPU as 16Mghz
# -mmcu=atmega328p   : target MCU
# -F                 : skip device signature read
# -V                 : disable automatic verify step
# -v                 : enable verbose output
# -D                 : disable auto erase flash

main.hex : main.c
	# compile to object file (optimize for size)
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=$(MMCU)  -o main.elf main.c twi.c
	# link as ELF binary
	#avr-gcc -mmcu=atmega328p main.o -o main.elf
	# convert ELF format to an IHEX format as used by avrdude
	avr-objcopy -O ihex -R .eeprom main.elf main.hex


burn:
	avrdude $(SKIP_CHECKS) -v $(AVRD_CONF) -c$(PROG_DEV) -p$(PROG_MMCU) -P$(TTY) -b 115200 $(DISABLE_AUTO_ERASE) -U flash:w:main.hex:i

clean :
	rm main.o twi.o main.elf main.hex
