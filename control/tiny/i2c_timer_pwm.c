// This program acts as the device (slave) for the control program i2c/a2a/c_ctl
#define F_CPU 8000000L
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usiTwiSlave.h"

#define I2C_SLAVE_ADDRESS 0x8 // the 7-bit address (remember to change this when adapting this example)

enum
{
 kCS13_10_idx = 0,  // Timer 1 Prescalar (CS13,CS12,CS11,CS10) from Table 12-5 pg 89
 kOCR1C_idx   = 1,  // OCR1C timer match value
 
};

volatile uint8_t ctl_regs[] =
{
 0x0f,   // clk/16384
 244,    // OCR1C
};

// Tracks the current register pointer position
volatile uint8_t reg_position = 0;
const uint8_t reg_size = sizeof(ctl_regs);

ISR(TIMER1_OVF_vect)
{
  PINB = _BV(PINB4);  // writes to PINB toggle the pins
}

void timer_init()
{
  TIMSK  &= ~_BV(TOIE1);    // Disable interrupt TIMER1_OVF
  OCR1A   = 255;            // Set to anything greater than OCR1C (the counter never gets here.)
  TCCR1  |= _BV(CTC1);      // Reset TCNT1 to 0 when TCNT1==OCR1C 
  TCCR1  |= _BV(PWM1A);     // Enable PWM A
  TCCR1  |= ctl_regs[kCS13_10_idx] & 0x0f;
  OCR1C   = ctl_regs[kOCR1C_idx];
  TIMSK  |= _BV(TOIE1);     // Enable interrupt TIMER1_OVF  
}



/**
 * This is called for each read request we receive, never put more
 * than one byte of data (with TinyWireS.send) to the send-buffer when
 * using this callback
 */
void on_request()
{
  // read and transmit the requestd position
  usiTwiTransmitByte(ctl_regs[reg_position]);

  
  // Increment the reg position on each read, and loop back to zero
  reg_position++;
  if (reg_position >= reg_size)
  {
    reg_position = 0;
  }
  
}


/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start,
 * data, restart/stop) on the bus does so be quick, set flags for long
 * running tasks to be called from the mainloop instead of running
 * them directly,
 */

void on_receive( uint8_t howMany )
{
    if (howMany < 1)
    {
        // Sanity-check
        return;
    }
    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }

    // get the register index to read/write
    reg_position = usiTwiReceiveByte();
    
    howMany--;

    // If only one byte was received then this was a read request
    // and the buffer pointer (reg_position) is now set to return the byte
    // at this location on the subsequent call to on_request() ...
    if (!howMany)
    {
      return;
    }

    // ... otherwise this was a write request and the buffer
    // pointer is now pointing to the first byte to write to
    while(howMany--)
    {
        ctl_regs[reg_position] = usiTwiReceiveByte();

        if(reg_position == kCS13_10_idx || reg_position == kOCR1C_idx )
          timer_init();
        
        reg_position++;
        if (reg_position >= reg_size)
        {
            reg_position = 0;
        }
    }
  
}



int main(void)
{
  cli();        // mask all interupts
    
  DDRB  |= _BV(DDB4);  // setup PB4 as output
  PORTB &= ~_BV(PINB4);

  timer_init();

  // setup i2c library
  usi_onReceiverPtr = on_receive; //on_receive;
  usi_onRequestPtr = on_request;
  usiTwiSlaveInit(I2C_SLAVE_ADDRESS);
  
  sei();

  PINB = _BV(PINB4);  // writes to PINB toggle the pins
  _delay_ms(1000);  
  PINB = _BV(PINB4);  // writes to PINB toggle the pins

  
  while(1)
  {
    //_delay_ms(1000);

    if (!usi_onReceiverPtr)
    {
        // no onReceive callback, nothing to do...
      continue;
    }
    
    if (!(USISR & ( 1 << USIPF )))
    {
        // Stop not detected
      continue;
    }

    
    uint8_t amount = usiTwiAmountDataInReceiveBuffer();
    if (amount == 0)
    {
        // no data in buffer
      continue;
    }

    
    usi_onReceiverPtr(amount);

    
  }
  return 0;
}



