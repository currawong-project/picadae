# ATtiny85 I2C protocol


Arguments in curly braces are optional.

------------------------------------------------------------------------------------------

__Hold PWM Duty Cycle__

Function                 | Opcode | Arguments
-------------------------|--------|---------------------------
Set hold PWM duty cycle  |    0   | (duty) { (freq) { (div) }}

Arguments | Range | Default | Note
----------|-------|---------|------------------------------------------------------
(duty)    | 0-255 |    127  | 0=0%, 255=100%
(freq)    | 0-255 |    255  | Sets PWM top value
(div)     | 1-15  |      5  | Set the PWM base clock frequency (See table below) 

PWM frequency and period for each possible (div) setting.

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

------------------------------------------------------------------------------------------

__Note-on Velocity__

Function                 | Opcode | Arguments
-------------------------|--------|---------------------------
Note on velocity         |    1   | (vel)

Arguments | Range | Default | Note
----------|-------|---------|------------------------------------------------------
(vel)     | 0-127 |  n/a    | Note on velocity.

Execute a note onset.
The (vel) value is translated to an attack pulse duration
by looking up the pulse tick count in the velocity table.

------------------------------------------------------------------------------------------

__Note-on Microseconds__

Function                 | Opcode | Arguments
-------------------------|--------|-------------------------------------------------
Note on ticks            |   2    | (pulse-ticks-high-byte) (pulse-ticks-low-byte)

Execute a note onset.
The 16 bit attack duration in ticks is calculated from microseconds.

    pulse-ticks = (usec / 1e6) * (16e6/256)

Where 16e6 is the system clock frequency and 256 is the timer0 clock divider.
`TCCR0B:C02,C01,C00 = 100b (4)`


ticks to micoseconds:

    usecs = (pulse-ticks * 1e6)*(256/16e6)


------------------------------------------------------------------------------------------

__Note-off__

Function                 | Opcode | Arguments
-------------------------|--------|---------------------------
Note off                 |   3    | None

Turn off a sounding note by settting the hold-voltage to 0.


------------------------------------------------------------------------------------------

__Set Read Address__

Function                 | Opcode | Arguments
-------------------------|--------|---------------------------
Set read address         |    4   | (src) {(addr)}

Set the source and address of the next I2C read request.

The read can come from one of three memory banks:
Register File, MIDI velocity table or EEPROM.
See the _Memory Location Id_ table below for the (src) id values.

Arguments   | Range | Default | Note
------------|-------|---------|-------------------------------------------------------
(src)       | 0-2   |  n/a    | Memory location id. See _Memory Location Id_ table.
(addr)      | 0-255 |  n/a    | Offset from base address set by (src)


------------------------------------------------------------------------------------------

__Write Memory__

Function                 | Opcode | Arguments
-------------------------|--------|---------------------------
Write memory             |    5   |


------------------------------------------------------------------------------------------

__Set Hold Delay__

Function                 | Opcode | Arguments
-------------------------|--------|---------------------------
Set hold delay           |    6   | { high {low}}

Set the length of the delay, in ticks, between when the attack pulse ends and when the
hold voltage is applied.
The high and low byte values are calculated identically to the
attack pulse duration values.

------------------------------------------------------------------------------------------

__Set *flags* variable__

Function                 | Opcode | Arguments
-------------------------|--------|---------------------------
Set flags variable       |    7   | (flags)

  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  
-----|-----|-----|-----|-----|-----|-----|-----
 n/a | n/a | n/a | n/a | n/a | n/a | n/a | HOA

_HOA_ : Set to apply the hold voltage at the beginning of the attack.

------------------------------------------------------------------------------------------

## Register File Address


Address | Label           | Note
--------|-----------------|------------------------------------------------------------------------------------------------
0       | Reg_Rd_Addr     | Next register file address to read.
1       | Table_Rd_Addr   | Next MIDI velocity address to read.
2       | EEPROM_Rd_Addr  | Next EEPROM address to read.
3       | Read_Src_Addr   | Source of the next I2C read request. See Memory Location Id table below.
4       | Reg_Wr_Addr     | Next register file address to write.
5       | Table_Rd_Addr   | Next MIDI velocity address to write.
6       | EEPROM_Rd_Addr  | Next EEPROM address to write.
7       | Write_Dst_Addr  | Destination of the next 'write operation'. See Memory Location id table below.
8       | Tmr_High_Addr   | Current attack pulse tick count high byte
9       | Tmr_Low_Addr    | Current attack pulse tick count low byte
10      | Tmr_Div_Addr    | Attack pulse counter frequency divider.
11      | Pwm_Duty_Addr   | Duty cycle of the hold PWM generator
12      | Pwm_Freq_Addr   | PWM counter max values. Determines the PWM frequency.
13      | Pwm_Div_Addr    | PWM clock divider
14      | State_Addr      | Current solenoied state.
15      | Error_Addr      | Error status
16      | Max_Tmr_Hi_Addr | Max attack pulse high byte tick count.
17      | Delay_High_Addr | Hold delay onset tick count high byte
18      | Delay_Low_Addr  | Hold delay onset tick count low byte
19      | Flags_Addr      | Binary variable field.


------------------------------------------------------------------------------------------

## Memory Location Id table

Id | Memory         | Note
---|----------------|-------------------------------
 0 | Register file  | See register table file
 1 | Velocity table | MIDI velocity to pulse ticks lookup table
 2 | EEPROM         | EEPROM data memory




