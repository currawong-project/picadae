# ATmega328 Interface Unit

This MCU acts as a serial to I2C translator for the host
computer.  The serial protocol implements two
interfaces.  One for read requests and another for write commands.

The data format of the write commands is given
in the ATtiny85 I2C protocol document.

Read requests return blocks of memory begining with the address
specified in the last 'Set read address' command.

## Read: Channel to Host

Read a `<count>` bytes from a channel beginning at offset = 'offset'

    'r' <i2c-addr> <offset> <count>

## Write: Host to Channel

Write `<count>` bytes from from the host to a channel

    'w' <i2c-addr> <register> <count> <value-0> <value-1> ... <value-n>
