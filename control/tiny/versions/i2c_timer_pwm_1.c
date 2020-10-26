//| Copyright: (C) 2018-2020 Kevin Larke <contact AT larke DOT org>
//| License: GNU GPL version 3.0 or above. See the accompanying LICENSE file.

/*                                    
                                    AT TINY 85
                                     +--\/--+
                              RESET _| 1  8 |_ +5V
             ~OC1B       HOLD  DDB3 _| 2  7 |_ SCL
              OC1B      ONSET  DDB4 _| 3  6 |_ DDB1 LED
                                GND _| 4  5 |_ SDA
                                     +------+
        * = Serial and/or programming pins on Arduino as ISP
*/


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
 kTmr0_Prescale_idx =  0,  // Timer 0 clock divider: 1=1,2=8,3=64,4=256,5=1024
 kTmr0_Coarse_idx   =  1,  // 
 kTmr0_Fine_idx     =  2,  //
 kPWM0_Duty_idx     =  3,  //
 kPWM0_Freq_idx     =  4,  // 1-4 = clock divider=1=1,2=8,3=64,4=256,5=1024 
 kCS13_10_idx       =  5,  // Timer 1 Prescalar (CS13,CS12,CS11,CS10) from Table 12-5 pg 89 (0-15)  prescaler = pow(2,val-1), 0=stop,1=1,2=2,3=4,4=8,....14=8192,15=16384  pre_scaled_hz = clock_hz/value
 kTmr1_Coarse_idx   =  6,  // count of times timer0 count to 255 before OCR1C is set to Tmr0_Fine
 kTmr1_Fine_idx     =  7,  // OCR1C timer match value
 kPWM1_Duty_idx     =  8,  //
 kPWM1_Freq_idx     =  9,  // 
 kTable_Addr_idx    = 10,  // Next table address to read/write 
 kTable_Coarse_idx  = 11,  // Next table coarse value to read/write
 kTable_Fine_idx    = 12,  // Next table fine value to read/write
 kMax_idx
};


volatile uint8_t ctl_regs[] =
{
   4,    //  0 (1-5)    4=32us per tick
 123,    //  1 (0-255)  Timer 0 Coarse Value 
   8,    //  2 (0-255)  Timer 0 Fine Value
 127,    //  3 (0-255) Duty cycle
   4,    //  4 (1-4)   PWM Frequency (clock pre-scaler)   
   9,    //  5  9=32 us period w/ 8Mhz clock (timer tick rate) 
 123,    //  6 (0-255) Tmr1_Coarse count of times timer count to 255 before loading Tmr0_Minor for final count.
   8,    //  7 (0-254) Tmr1_Fine OCR1C value on final phase before triggering timer
 127,    //  8 (0-255) PWM1 Duty cycle
 254,    //  9 (0-255) PWM1 Frequency  (123 hz)
   0,    // 10 (0-127) Next table addr to read/write
   0,    // 11 (0-255) Next table coarse value to write
   0,    // 12 (0-255) Next table fine value to write
};

#define tableN 256
uint8_t table[ tableN ];
 

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// EEPROM
//

void EEPROM_write(uint8_t ucAddress, uint8_t ucData)
{
  // Wait for completion of previous write 
  while(EECR & (1<<EEPE))
  {}
    
  EECR = (0<<EEPM1)|(0<<EEPM0); // Set Programming mode   
  EEAR = ucAddress;             // Set up address and data registers 
  EEDR = ucData;  
  EECR |= (1<<EEMPE);           // Write logical one to EEMPE 
  EECR |= (1<<EEPE);            // Start eeprom write by setting EEPE 
}


uint8_t EEPROM_read(uint8_t ucAddress)
{
  // Wait for completion of previous write 
  while(EECR & (1<<EEPE))
  {}
    
  EEAR = ucAddress;  // Set up address register 
  EECR |= (1<<EERE); // Start eeprom read by writing EERE 
  return EEDR;       // Return data from data register 
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Read/Write table
//

// To write table value 42 to 127 (coarse) 64 (fine)
//
// w 8 kTable_Addr_idx 42
// w 8 kTable_Coarse_idx 127
// w 8 kTable_fine_idx 64
//
// TO read table value 42
// w 8 kTable_Addr_idx 42
// r 8 kTable_Coarse_idx -> 127
// r 8 kTable_Fine_idx   ->  64


#define eeprom_addr( addr ) (kMax_idx + (addr))

void table_write_cur_value( void )
{
  uint8_t tbl_addr = ctl_regs[ kTable_Addr_idx ] * 2;
  
  table[ tbl_addr+0 ] = ctl_regs[ kTable_Coarse_idx ];
  table[ tbl_addr+1 ] = ctl_regs[ kTable_Fine_idx ];

  EEPROM_write( eeprom_addr( tbl_addr+0 ), ctl_regs[ kTable_Coarse_idx ] );
  EEPROM_write( eeprom_addr( tbl_addr+1 ), ctl_regs[ kTable_Fine_idx ]); 
}

void table_load( void )
{
  uint8_t i = 0;

  for(; i<128; ++i)
  {
    uint8_t tbl_addr  = i*2;
    table[tbl_addr+0] = EEPROM_read( eeprom_addr(tbl_addr+0) );
    table[tbl_addr+1] = EEPROM_read( eeprom_addr(tbl_addr+1) );
  }  
}

void restore_memory_from_eeprom( void )
{
  /*
  uint8_t i;
  for(i=0; i<kMax_idx; ++i)
  {
    ctl_regs[i] = EEPROM_read( eeprom_addr( i ) );
  }
  */
  
  table_load();
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Timer0
//

volatile uint8_t tmr0_state        = 0;    // 0=disabled 1=coarse mode, 2=fine mode 
volatile uint8_t tmr0_coarse_cur   = 0;

// Use the current tmr0 ctl_reg[] values to set the timer to the starting state.
void tmr0_reset()
{
  // if a coarse count exists then go into coarse mode 
  if( ctl_regs[kTmr0_Coarse_idx] > 0 )
  {
    tmr0_state = 1;
    OCR0A      = 0xff;
  }
  else // otherwise go into fine mode
  {
    tmr0_state = 2;
    OCR0A     = ctl_regs[kTmr0_Fine_idx];
  }
  
  tmr0_coarse_cur = 0;  
}

ISR(TIMER0_COMPA_vect)
{
  switch( tmr0_state )
  {
    case 0:
      // disabled
      break;

    case 1: 
      // coarse mode
      if( ++tmr0_coarse_cur >= ctl_regs[kTmr0_Coarse_idx] )
      {
        tmr0_state  = 2;
        OCR0A     = ctl_regs[kTmr0_Fine_idx];        
      }
      break;

    case 2:
      // fine mode
      PINB = _BV(PINB4);  // writes to PINB toggle the pins

      tmr0_reset(); // restart the timer 
      break;
  }
}


void timer0_init()
{
  TIMSK  &= ~_BV(OCIE0A);    // Disable interrupt TIMER1_OVF
  TCCR0A  |=  0x02;           // CTC mode
  TCCR0B  |= ctl_regs[kTmr0_Prescale_idx]; // set the prescaler

  GTCCR  |= _BV(PSR0);      // Set the pre-scaler to the selected value
  
  tmr0_reset();              // set the timers starting state
  
  TIMSK  |= _BV(OCIE0A);     // Enable interrupt TIMER1_OVF  

}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// PWM (Timer0)
//

void pwm0_update()
{
  OCR0B   = ctl_regs[kPWM0_Duty_idx];  // 50% duty cycle
  TCCR0B |= ctl_regs[kPWM0_Freq_idx]; // PWM frequency pre-scaler
}

void pwm0_init()
{
  // WGM[1:0] = 3 (TOP=255)
  // OCR0B = duty cycle (0-100%)
  // COM0A[1:0] = 2 non-inverted
  //

  TCCR0A |=  0x20   + 3;    // 0x20=non-inverting 3=WGM bits Fast-PWM mode (0=Bot 255=Top)
  TCCR0B |=  0x00   + 4;    //                    3=256 pre-scaler  122Hz=1Mghz/(v*256) where v=64
  
  GTCCR  |= _BV(PSR0);      // Set the pre-scaler to the selected value

  pwm0_update();

  
  DDRB |= _BV(DDB1);    // set direction on 
}



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Timer1
//

volatile uint8_t tmr1_state        = 0;    
volatile uint8_t tmr1_coarse_cur   = 0;
static   uint8_t tmr1_init_fl      = 0;

void tmr1_reset()
{
  if( ctl_regs[kTmr1_Coarse_idx] > 0 )
  {
    tmr1_state = 1;
    OCR1C     = 254;
  }
  else
  {
    tmr1_state = 2;
    OCR1C     = ctl_regs[kTmr1_Fine_idx];
  }
  
  tmr1_coarse_cur = 0;  
}

ISR(TIMER1_OVF_vect)
{
  if( !tmr1_init_fl )
  {
    PORTB |= _BV(PINB3);  // set PWM pin
  }
  else
  {
    switch( tmr1_state )
    {
    
      case 0:
        // disabled
        break;

      case 1:
        // coarse mode
        if( ++tmr1_coarse_cur >= ctl_regs[kTmr1_Coarse_idx] )
        {
          tmr1_state  = 2;
          OCR1C     = ctl_regs[kTmr1_Fine_idx];        
        }
        break;

      case 2:
        // fine mode
        PINB = _BV(PINB4);  // writes to PINB toggle the pins

        tmr1_reset();
        break;
    }
  } 
}

void timer1_init()
{
  TIMSK  &= ~_BV(TOIE1);    // Disable interrupt TIMER1_OVF
  OCR1A   = 255;            // Set to anything greater than OCR1C (the counter never gets here.)
  TCCR1  |= _BV(CTC1);      // Reset TCNT1 to 0 when TCNT1==OCR1C 
  TCCR1  |= _BV(PWM1A);     // Enable PWM A (to generate overflow interrupts)
  TCCR1  |= ctl_regs[kCS13_10_idx] & 0x0f;  // 
  GTCCR  |= _BV(PSR1);      // Set the pre-scaler to the selected value

  tmr1_reset();
  tmr1_init_fl = 1;
  TIMSK  |= _BV(TOIE1);     // Enable interrupt TIMER1_OVF  
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// PWM1
//
// PWM is optimized to use pins OC1A ,~OC1A, OC1B, ~OC1B but this code
// but since these pins are not available this code uses
// ISR's to redirect the output to PIN3

void pwm1_update()
{
  OCR1B   = ctl_regs[kPWM1_Duty_idx]; // control duty cycle
  OCR1C   = ctl_regs[kPWM1_Freq_idx]; // PWM frequency pre-scaler
}

ISR(TIMER1_COMPB_vect)
{
  PORTB &= ~(_BV(PINB3)); // clear PWM pin
}


void pwm1_init()
{
  TIMSK  &= ~(_BV(OCIE1B) + _BV(TOIE1));    // Disable interrupts
  
  DDRB   |=  _BV(DDB3);  // setup PB3 as output  

  // set on TCNT1 == 0     // happens when TCNT1 matches OCR1C
  // clr on OCR1B == TCNT  // happens when TCNT1 matches OCR1B
  //                       // COM1B1=1 COM1B0=0 (enable output on ~OC1B)
  TCCR1  |= 9;             // 32us period (256 divider) prescaler
  GTCCR  |= _BV(PWM1B);    // Enable PWM B and disconnect output pins
  GTCCR  |= _BV(PSR1);     // Set the pre-scaler to the selected value

  pwm1_update();

  TIMSK  |= _BV(OCIE1B) + _BV(TOIE1);    // Enable interrupts


  
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// Tracks the current register pointer position
volatile uint8_t reg_position = 0;
const uint8_t    reg_size = sizeof(ctl_regs);

//
// Read Request Handler
//
// This is called for each read request we receive, never put more
// than one byte of data (with TinyWireS.send) to the send-buffer when
// using this callback
//
void on_request()
{
  uint8_t val = 0;
  
  switch( reg_position )
  {
    case kTable_Coarse_idx:
      val = table[ ctl_regs[kTable_Addr_idx]*2 + 0 ];      
      break;

    case kTable_Fine_idx:
      val = table[ ctl_regs[kTable_Addr_idx]*2 + 1 ];
      break;
      
    default:
      // read and transmit the requestd position
      val = ctl_regs[reg_position];

  }
  
  usiTwiTransmitByte(val);
  
  // Increment the reg position on each read, and loop back to zero
  reg_position++;
  if (reg_position >= reg_size)
  {
    reg_position = 0;
  }
  
}


//
// The I2C data received -handler
//
// This needs to complete before the next incoming transaction (start,
// data, restart/stop) on the bus does so be quick, set flags for long
// running tasks to be called from the mainloop instead of running
// them directly,
//

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
      // write the value
      ctl_regs[reg_position] = usiTwiReceiveByte();

      // Set timer 1
      if( kTmr0_Prescale_idx <= reg_position && reg_position <= kTmr0_Fine_idx )
      { timer0_init(); }
      else

          
        // Set PWM 0
        if( kPWM0_Duty_idx <= reg_position && reg_position <= kPWM0_Freq_idx )
        { pwm0_update(); }
        else
        
          // Set timer 1
          if( kCS13_10_idx <= reg_position && reg_position <= kTmr1_Fine_idx )
          { timer1_init(); }
          else

            // Set PWM 1
            if( kPWM1_Duty_idx <= reg_position && reg_position <= kPWM1_Freq_idx )
            { pwm1_update(); }
            else
          

              // Write table
              if( reg_position == kTable_Fine_idx )
              { table_write_cur_value(); }
        
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


  restore_memory_from_eeprom();
  
  DDRB  |=   _BV(DDB4)  + _BV(DDB3)  + _BV(DDB1);  // setup PB4,PB3,PB1 as output  
  PORTB &= ~(_BV(PINB4) + _BV(PINB3) + _BV(PINB1)); // clear output pins

  
  timer0_init();
  pwm1_init();
  
  // setup i2c library
  usi_onReceiverPtr = on_receive; 
  usi_onRequestPtr  = on_request;
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



