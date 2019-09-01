/*                                    
                                    AT TINY 85
                                     +--\/--+
                              RESET _| 1  8 |_ +5V
                         HOLD  DDB3 _| 2  7 |_ SCL
                        ONSET  DDB4 _| 3  6 |_ DDB1 LED
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
 kCS13_10_idx     = 0,  // Timer 1 Prescalar (CS13,CS12,CS11,CS10) from Table 12-5 pg 89 (0-15)  prescaler = pow(2,val-1), 0=stop,1=1,2=2,3=4,4=8,....14=8192,15=16384  pre_scaled_hz = clock_hz/value
 kTmr0_Coarse_idx = 1,  // count of times timer0 count to 255 before OCR1C is set to Tmr0_Fine
 kTmr0_Fine_idx   = 2,  // OCR1C timer match value
 kPWM_Duty_idx    = 3,  //
 kPWM_Freq_idx    = 4,  // 1-4 = clock divider=1=1,2=8,3=64,4=256,5=1024
 kTable_Addr_idx  = 5,  // Next table address to write 
 kTable_Coarse_idx= 6,  // Next table coarse value to write
 kTable_Fine_idx  = 7,  // Next table fine value to write
 kMax_idx
};

volatile uint8_t ctl_regs[] =
{
   9,    // 0  9=32 us period w/ 8Mhz clock (timer tick rate) 
 123,    // 1 (0-255) Tmr0_Coarse count of times timer count to 255 before loading Tmr0_Minor for final count.
   8,    // 2 (0-254) Tmr0_Fine OCR1C value on final phase before triggering timer
 127,    // 3 (0-255) Duty cycle
   4,    // 4 (1-4)   PWM Frequency (clock pre-scaler)
   0,    // 7 (0-127) Next table addr to read/write
   0,    // 5 (0-255) Next table coarse value to write
   0,    // 6 (0-255) Next table fine value to write
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

#define eeprom_addr( addr ) (kMax_idx + (addr))

void table_write( void )
{
  uint8_t addr = ctl_regs[ kTable_Addr_idx ] * 2;
  
  table[ addr+0 ] = ctl_regs[ kTable_Coarse_idx ];
  table[ addr+1 ] = ctl_regs[ kTable_Fine_idx ];

  EEPROM_write( eeprom_addr( addr+0 ), ctl_regs[ kTable_Coarse_idx ] );
  EEPROM_write( eeprom_addr( addr+1 ), ctl_regs[ kTable_Fine_idx ]); 
}

void table_load( void )
{
  uint8_t i = 0;

  for(; i<128; ++i)
  {
    uint8_t addr  = i*2;
    table[addr+0] = EEPROM_read( eeprom_addr(addr) );
    table[addr+1] = EEPROM_read( eeprom_addr(addr) );
  }  
}

void restore_memory_from_eeprom( void )
{
  uint8_t i;
  for(i=0; i<kMax_idx; ++i)
  {
    ctl_regs[i] = eeprom_read( eeprom_addr( i ) );
  }
      
  table_load();
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// PWM (Timer0)
//

void pwm0_update()
{
  OCR0B   = ctl_regs[kPWM_Duty_idx];  // 50% duty cycle
  TCCR0B |= ctl_regs[kPWM_Freq_idx]; // PWM frequency pre-scaler
}

void pwm0_init()
{
  // WGM[1:0] = 3 (TOP=255)
  // OCR0B = duty cycle (0-100%)
  // COM0A[1:0] = 2 non-inverted
  //

  TCCR0A |=  0x20   + 3;    // 0x20=non-inverting 3=WGM bits Fast-PWM mode (0=Bot 255=Top)
  TCCR0B |=  0x00   + 4;    //                    3=256 pre-scaler  122Hz=1Mghz/(v*256) where v=64

  pwm0_update();
    
  DDRB |= _BV(DDB1);    
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Timer1
//

volatile uint8_t tmr_state        = 0;    
volatile uint8_t tmr_coarse_cur   = 0;

void tmr_reset()
{
  if( ctl_regs[kTmr0_Coarse_idx] > 0 )
  {
    tmr_state = 1;
    OCR1C     = 254;
  }
  else
  {
    tmr_state = 2;
    OCR1C     = ctl_regs[kTmr0_Fine_idx];
  }
  
  tmr_coarse_cur = 0;  
}

ISR(TIMER1_OVF_vect)
{
  switch( tmr_state )
  {
    case 0:
      break;
      
    case 1:
      if( ++tmr_coarse_cur >= ctl_regs[kTmr0_Coarse_idx] )
      {
        tmr_state  = 2;
        OCR1C     = ctl_regs[kTmr0_Fine_idx];        
      }
      break;
      
    case 2:
      PINB = _BV(PINB4) + _BV(PINB1);  // writes to PINB toggle the pins

      tmr_reset();
      break;
  }
}

void timer1_init()
{
  TIMSK  &= ~_BV(TOIE1);    // Disable interrupt TIMER1_OVF
  OCR1A   = 255;            // Set to anything greater than OCR1C (the counter never gets here.)
  TCCR1  |= _BV(CTC1);      // Reset TCNT1 to 0 when TCNT1==OCR1C 
  TCCR1  |= _BV(PWM1A);     // Enable PWM A
  TCCR1  |= ctl_regs[kCS13_10_idx] & 0x0f;  // 
  GTCCR  |= _BV(PSR1);      // Set the pre-scaler to the selected value

  tmr_reset();
  
  TIMSK  |= _BV(TOIE1);     // Enable interrupt TIMER1_OVF  
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
      val = table[ reg_position*2 + 0 ];      
      break;

    case kTable_Fine_idx:
      val = table[ reg_position*2 + 1 ];
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
        ctl_regs[reg_position] = usiTwiReceiveByte();

        // Set timer  
        if( kCS13_10_idx <= reg_position && reg_position <= kTmr0_Fine_idx )
        { timer1_init(); }
        else

        // Set PWM
        if( kPWM_Duty_idx <= reg_position && reg_position <= kPWM_Freq_idx )
        { pwm0_update(); }
        else

        // Write table
        if( reg_position == kTable_Fine_idx )
        { table_write(); }
        
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



