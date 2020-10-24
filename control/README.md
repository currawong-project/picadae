This folder contains the _picadae_ control code.

The _picadae_ control program consists of three parts:

- _tiny_ : Keyboard solenoid driver
- _ctrl_ : Keyboard central controller
- _app_  : Monitor and command utility.

# Architecture

The keyboard control hardware consists of 8 driver boards
mounted above the solenoids on the keyboard assembly.
Each driver board has 11 key driver circuits and can
therefore control 11 piano keys. Each key driver
circuit consists of an ATtiny85 microcontroller
which controls a set of power transistors
which in turn control the state of the solenoid.
The solenoid can be in one of three states:

- off
- attack
- hold 

The attack state is triggered by pulsing the 
the solenoid with a fast (250 microsecond to
30 millisecond) 36 volt pulse. The duration
of the pulse determines the attack dynamic.
The hold state then holds the key down
during the sustain portion of the note.

All 88 ATtiny85 microncontrollers are attached
to a single I2C bus which is shared with
an Arduino Uno.  The application computer
communicate with the Uno via 
a serial connection. The Uno in turn
translates messages between the 
ATtiny85 channel computers and the
application computer.

# Keyboard central controller : Arduino Uno Serial Interface

Read a register value of `<count>` bytes from channel `<i2c-addr>`

    'r' <i2c-addr> <register> <count>
	
Write a register value of `<count>` bytes from channel `<i2c-addr>`

	'w' <i2c-addr> <register> <count> <value-0> <value-1> ... <value-n>


# Keyboard solenoid driver : ATtiny85 Firmware


PWM counter frequency and period.
for each possible `div` setting
with 16 Mhz system clock.

Value | Div  | Frequency   | Period
------|------|-------------|-------
 1    |    1 |    16    M  | 62.5 n 
 2    |    2 |     8    M  |  125 n
 3    |    4 |     4    M  |  250 n
 4    |    8 |     2    M  |  500 n
 5    |   16 |     1    M  |    1 u
 6    |   32 |   500    K  |    2 u
 7    |   64 |   250    K  |    4 u
 8    |  128 |   125    K  |    8 u
 9    |  256 | 62500    Hz |   16 u
10    |  512 | 31250    Hz |   32 u
11    | 1024 | 15625    Hz |   64 u
12    | 2048 | 7812.5   Hz |  128 u
13    | 4096 | 3906.25  Hz |  256 u
14    | 8192 | 1953.125 Hz |  512 u
15    |16384 | 976.6625 Hz | 1024 u 


# Monitor and command utility : Python Utility






