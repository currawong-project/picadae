#
# Usage: make I2C_ADDR=8
#

ifndef TTY
TTY=/dev/ttyACM0
endif

ifndef TARGET
TARGET=i2c_timer_pwm_2
endif

MCU=attiny85
AVRDUDEMCU=t85
CC=/usr/bin/avr-gcc
CFLAGS=-g -Os -Wall -mcall-prologues -mmcu=$(MCU) -DI2C_SLAVE_ADDRESS=$(I2C_ADDR)
OBJ2HEX=/usr/bin/avr-objcopy
AVRDUDE=avrdude

# See http://www.engbedded.com/fusecalc for fuse settings
# /usr/bin/avrdude -C/etc/avrdude/avrdude.conf -v -pattiny85 -cstk500v1 -P/dev/ttyACM0 -b19200 -Uflash:w:/tmp/arduino_build_108059/i2c.ino.hex:i 
# 	

# lfuse=0xe2 =  8 Mghz
# lfuse=0xe1 = 16 Mghz

all:
	$(CC) $(CFLAGS) $(TARGET).c usiTwiSlave.c -o$(TARGET)
	$(OBJ2HEX) -R .eeprom -O ihex $(TARGET) $(TARGET).hex

burn:
	$(AVRDUDE) -p $(MCU) -P $(TTY)  -C/etc/avrdude/avrdude.conf -v -c avrisp -b 19200 -U flash:w:$(TARGET).hex -U lfuse:w:0xe1:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m
clean:
	rm -f *.hex *.obj *.o

