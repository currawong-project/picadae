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
 kCS13_10_idx     = 0,  // Timer 1 Prescalar (CS13,CS12,CS11,CS10) from Table 12-5 pg 89
 kTmr0_Coarse_idx = 1,  // count of times timer0 count to 255 before OCR1C is set to Tmr0_Minor
 kTmr0_Fine_idx   = 2,  // OCR1C timer match value
 kPWM_Duty_idx    = 3,  //
 kPWM_Freq_idx    = 4,  // 1-4 = clock divider=1=1,2=8,3=64,4=256,5=1024
};

volatile uint8_t ctl_regs[] =
{
 0x0f,   // 0 (0-15)  timer prescalar 0=stop, prescaler = pow(2,val-1), 0=stop,1=1,2=2,3=4,4=8,....14=8192,15=16384  pre_scaled_hz = clock_hz/value
 0,      // 1 (0-255) Tmr0_Coarse count of times timer count to 255 before loading Tmr0_Minor for final count.
 244,    // 2 (0-254) Tmr0_Fine OCR1C value on final phase before triggering timer
 127,    // 3 (0-255) Duty cycle
 4,      // 4 (1-4)   PWM Frequency (clock pre-scaler) 
};

// Tracks the current register pointer position
volatile uint8_t reg_position = 0;
const uint8_t reg_size = sizeof(ctl_regs);

ISR(TIMER1_OVF_vect)
{
  PINB = _BV(PINB4) + _BV(PINB1);  // writes to PINB toggle the pins
  
}


void timer1_init()
{
  TIMSK  &= ~_BV(TOIE1);    // Disable interrupt TIMER1_OVF
  OCR1A   = 255;            // Set to anything greater than OCR1C (the counter never gets here.)
  TCCR1  |= _BV(CTC1);      // Reset TCNT1 to 0 when TCNT1==OCR1C 
  TCCR1  |= _BV(PWM1A);     // Enable PWM A
  TCCR1  |= ctl_regs[kCS13_10_idx] & 0x0f;  // 
  OCR1C   = ctl_regs[kTmr0_Fine_idx];
  TIMSK  |= _BV(TOIE1);     // Enable interrupt TIMER1_OVF  
}

void pwm0_update()
{
  OCR0B   = ctl_regs[kPWM_Duty_idx];  // 50% duty cycle
  TCCR0B |= ctl_regs[kPWM_Freq_idx]; // PWM frequency pre-scaler
}

void pwm0_init()
{
  //WGM[1:0] = 3 (TOP=255)
  // OCR0B = duty cycle (0-100%)
  // COM0A[1:0] = 2 non-inverted
  //

  TCCR0A |=  0x20   + 3;    // 0x20=non-inverting 3=WGM bits Fast-PWM mode (0=Bot 255=Top)
  TCCR0B |=  0x00   + 4;    //                    3=256 pre-scaler  122Hz=1Mghz/(v*256) where v=64

  pwm0_update();
    
  DDRB |= _BV(DDB1);    
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

void on_receive( uint8_t byteN )
{
    if (byteN < 1)
    {
        // Sanity-check
        return;
    }
    if (byteN > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }

    // get the register index to read/write
    reg_position = usiTwiReceiveByte();
    
    byteN--;

    // If only one byte was received then this was a read request
    // and the buffer pointer (reg_position) is now set to return the byte
    // at this location on the subsequent call to on_request() ...
    if (!byteN)
    {
      return;
    }

    // ... otherwise this was a write request and the buffer
    // pointer is now pointing to the first byte to write to
    while(byteN--)
    {
        ctl_regs[reg_position] = usiTwiReceiveByte();

        if( kCS13_10_idx <= reg_position && reg_position <= kTmr0_Fine_idx )
          timer1_init();

        if( kPWM_Duty_idx <= reg_position && reg_position <= kPWM_Freq_idx )
          pwm0_update();

        
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
    
  DDRB  |= _BV(DDB4) + _BV(DDB1);  // setup PB4 as output
  PORTB &= ~(_BV(PINB4) + _BV(PINB1));

  timer1_init();
  pwm0_init();
  
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



