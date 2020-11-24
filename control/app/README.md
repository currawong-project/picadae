# Picadea control and monitor shell


Shell commands:



key | Function         | Arguments                                  | Notes
----|------------------|--------------------------------------------|----------------------------------------------------------
q   | quit             |                                            |
?   | print usage      |                                            |
w   | write            | (i2c_addr) (reg_addr) (data0) ... (dataN)  |
r   | read             | (i2c_addr) (src) (reg_addr) (byteN)        | See [Memory Source Id Table](#memory-source-id-table).
v   | note-on          | (pitch) (vel)                              |
u   | note-on          | (pitch) (usec) (div)                       | See [Pulse Timer Divisor Table](#pulse-timer-divisor).
o   | note-off         | (pitch)                                    |
T   | set table        | (pitch) (vel) (usec)                       |
t   | get table        | (pitch) (vel)                              |
D   | set duty         | (pitch) (percent) {(hz) {(div)}}           |
d   | get duty         | (pitch)                                    |
H   | set hold delay   | (pitch) (usec)                             |
h   | get hold delay   | (pitch)                                    |
F   | set pwm freq     | (pitch) (hz)                               | 254=~123Hz
f   | get pwm freq     | (pitch)                                    |
I   | set pwm div      | (pitch) (div)                              | See [PWM Divisor Table](#pwm-divisor).
i   | get pwm div      | (pitch)                                    |
A   | set flags        | (pitch) (flags)                            |
a   | get flags        | (pitch)                                    |
W   | write table      | (pitch)                                    |
N   | note w/ duration | (pitch) (atkUs) (durMs)                    |
S   | seq              | (pitch) (atkUs) (durMs) (deltaUs) (noteN)  |
L   | log              | (level) (0-1)                              |



## PWM Divisor

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


## Pulse Timer Divisor

Value | Divisor | Frequency  | Period
------|---------|------------|--------------
  1   |      1  | 16000000   |     62.5n
  2   |      8  |  2000000   |    500.0n
  3   |     64  |   250000   |      4.0u
  4   |    256  |    62500   |     16.0u
  5   |   1024  |    15625   |     64.0u
  

## Memory Source Id Table

Id | Memory         | Note
---|----------------|-------------------------------
 0 | Register file  | See register table file
 1 | Velocity table | MIDI velocity to pulse ticks lookup table
 2 | EEPROM         | EEPROM data memory


## Memory Destination Id Table

Id | Memory         | Note
---|----------------|-------------------------------
 4 | Register file  | See register table file
 5 | Velocity table | MIDI velocity to pulse ticks lookup table
 6 | EEPROM         | EEPROM data memory
