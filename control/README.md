This folder contains the _picadae_ control code.

The _picadae_ control program consists of three parts:

- _tiny_ : [ATtiny85 solenoid driver firmware](tiny/README.md)

- _ctrl_ : [ATmega328 interface unit](ctrl/README.md) 

- _app_  : [Python monitor and command utility](app/README.md)

# Architecture

The keyboard control hardware consists of 8 driver boards mounted
above the solenoids on the keyboard assembly.  Each driver board has
11 key driver circuits which control 11 piano keys. Each key driver
circuit consists of an ATtiny85 microcontroller which controls a set
of power transistors which in turn control the state of the solenoid.
The solenoid can be in one of three states:

- off
- attack
- hold 

The attack state is triggered by pulsing the 
the solenoid with a fast (250 microsecond to
30 millisecond) 36 volt pulse. The duration
of the pulse determines the attack dynamic.
Longer pulses produce louder notes.
At the end of the attack state the solenoid
transitions into a hold state. During
this time the key is held down with just
enough force to sustain the note.

All 88 ATtiny85 microncontrollers are attached
to a single I2C bus which is mastered by an ATmega328 MCU
which acts as a serial to I2C translator for the host computer.

The host API is implemented as the Python class 'Picadae' in app/picadae_api.py.
The control shell program app/picadae_shell.py shows an example
usage of the API.



 












