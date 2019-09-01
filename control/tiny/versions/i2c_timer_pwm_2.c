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

#define HOLD_DIR DDB3
#define ATTK_DIR DDB4
#define LED_DIR  DDB1

#define HOLD_PIN PINB3
#define ATTK_PIN PINB4
#define LED_PIN  PINB1

// Opcodes
enum
{ 
 kSetReg_Op         =  0,  // Set register  <hi_addr> <lo_addr> <value0> ... <valueN>
 kSetPwm_Op         =  1,  // Set PWM registers <enable> <duty> <freq>
 kNoteOnVel_Op      =  2,  // Turn on note <vel>
 kNoteOnUsec_Op     =  3,  // Turn on note <coarse> <fine>
 kNoteOff_Op        =  4,  // Turn off note
 kRead_Op           =  5,  // Read a value {{ <src>} <addr> }
 kInvalid_Op        =  6
};


// Register addresses
enum
{
 kTmr_Coarse_idx     =  0,  // Current Timer 0 coarse count
 kTmr_Fine_idx       =  1,  // Current Timer 0 fine count
 kTmr_Prescale_idx   =  2,  // Current Timer 0 clock divider: 1=1,2=8,3=64,4=256,5=1024
 kPwm_Enable_idx     =  3,  // Current PWM 1 enable flag
 kPwm_Duty_idx       =  4,  // Current PWM 1 duty cycle
 kPwm_Freq_idx       =  5,  // Current PWM 1 frequency
 kRead_Src_idx       =  6,  // 0=reg, 1=table, 2=eeprom
 kReg_Addr_idx       =  7,  // Next Reg Address to read
 kTable_Addr_idx     =  8,  // Next Table Address to read
 kEE_Addr_idx        =  9,  // Next EEPROM address to read
 kError_Code_idx     = 10,  // Error Code
 kMax_idx
};

// Regster memory
volatile uint8_t ctl_regs[] =
{
 
 123,    //  1 (0-255)  Timer 0 Coarse Value 
   8,    //  2 (0-255)  Timer 0 Fine Value
   4,    //  0 (1-5)    4=32us per tick

   1,    //  5 (0-1)   PWM1 Enable 
 127,    //  3 (0-255) PWM1 Duty cycle (0-100%)
 254,    //  4 (0-255) PWM1 Frequency  (123 hz)

   0,    //  6 (0-255) Read Source 
   0,    //  7 (0-255) Reg addr   
   0,    //  8 (0-255) Table addr
   0,    //  9 (0-255) EEPROM addr

   0,    // 10 (0-255) Error code
   
};

volatile uint8_t table[128];



#define stackN 16 
volatile uint8_t stack[ stackN ];

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
    
  EECR  = (0<<EEPM1)|(0<<EEPM0);      // Set Programming mode   
  EEARH = 0;                          // Set up address and data registers
  EEARL = ucAddress;
  EEDR  = ucData;  
  EECR |= (1<<EEMPE);                // Write logical one to EEMPE 
  EECR |= (1<<EEPE);                 // Start eeprom write by setting EEPE 
}


uint8_t EEPROM_read(uint8_t ucAddress)
{
  // Wait for completion of previous write 
  while(EECR & (1<<EEPE))
  {}
    
  EEARH = 0;                          // Set up address and data registers
  EEARL = ucAddress;
  EECR |= (1<<EERE);                 // Start eeprom read by writing EERE 
  return EEDR;                       // Return data from data register 
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Read/Write table
//

#define eeprom_addr( addr ) (addr)


void table_load( void )
{
  uint8_t i = 0;

  for(; i<64; ++i)
  {
    uint16_t tbl_addr  = i*2;
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
  if( ctl_regs[kTmr_Coarse_idx] > 0 )
  {
    tmr0_state = 1;
    OCR0A      = 0xff;
  }
  else // otherwise go into fine mode
  {
    tmr0_state = 2;
    OCR0A     = ctl_regs[kTmr_Fine_idx];
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
      if( ++tmr0_coarse_cur >= ctl_regs[kTmr_Coarse_idx] )
      {
        tmr0_state  = 2;
        OCR0A     = ctl_regs[kTmr_Fine_idx];        
      }
      break;

    case 2:
      // fine mode
      PINB = _BV(ATTK_PIN);  // writes to PINB toggle the pins

      tmr0_reset(); // restart the timer 
      break;
  }
}


void timer0_init()
{
  TIMSK  &= ~_BV(OCIE0A);    // Disable interrupt TIMER0_COMPA_vect
  TCCR0A  |=  0x02;           // CTC mode
  TCCR0B  |= ctl_regs[kTmr_Prescale_idx]; // set the prescaler

  GTCCR  |= _BV(PSR0);      // Set the pre-scaler to the selected value
  
  tmr0_reset();              // set the timers starting state
  
  TIMSK  |= _BV(OCIE0A);     // Enable interrupt TIMER0_COMPA_vect

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
  OCR1B   = ctl_regs[kPwm_Duty_idx]; // control duty cycle
  OCR1C   = ctl_regs[kPwm_Freq_idx]; // PWM frequency pre-scaler
}

ISR(TIMER1_OVF_vect)
{
    PORTB |= _BV(HOLD_PIN);  // set PWM pin
}

ISR(TIMER1_COMPB_vect)
{
  PORTB &= ~(_BV(HOLD_PIN)); // clear PWM pin
}


void pwm1_init()
{
  TIMSK  &= ~(_BV(OCIE1B) + _BV(TOIE1));    // Disable interrupts
  
  DDRB   |=  _BV(HOLD_DIR);  // setup PB3 as output  

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


//
// Read Request Handler
//
// This is called for each read request we receive, never put more
// than one byte of data (with TinyWireS.send) to the send-buffer when
// using this callback
//

void on_request()
{
  uint8_t  val  = 0;

  switch( ctl_regs[ kRead_Src_idx ] )
  {
    case 0:
      val = table[ ctl_regs[ kReg_Addr_idx ] ];
      ctl_regs[ kReg_Addr_idx ] += 1;
      break;
      
    case 1:
      val = table[ ctl_regs[ kTable_Addr_idx ] ];
      ctl_regs[ kTable_Addr_idx ] += 1;
      break;
      
    case 2:
      val = EEPROM_read(ctl_regs[ kEE_Addr_idx]);
      ctl_regs[ kEE_Addr_idx ] += 1;      
  }
  
  usiTwiTransmitByte(val);

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
  uint8_t stack_idx = 0;

  PINB = _BV(LED_PIN);  // writes to PINB toggle the pins
  
  // Sanity-check
  if( byteN < 1 || byteN > TWI_RX_BUFFER_SIZE)
  {
    // TODO: signal an error
    return;
  }

  // get the command byte
  uint8_t cur_op_id = usiTwiReceiveByte();
    
  --byteN;

  // verify that cur_op_id is valid
  if( cur_op_id < kInvalid_Op )
  {
    // TODO: signal an error
    return;
  }
  
  // get the command arguments
  while(byteN--)
  {
    // write the value
    stack[stack_idx] = usiTwiReceiveByte();

    ++stack_idx;
        
    if(stack_idx >= stackN)
    {
      // TODO: signal an error
      break;
    }  
  }

  // execute the operation
  switch( cur_op_id )
  {
    case kSetReg_Op:      // Set register  <reg> <value0> ... <valueN>
      if( stack_idx > 1 )
      {
        uint8_t addr = stack[0];
        uint8_t i = 2;
        for(; i<stack_idx; ++i,++addr)
          ctl_regs[ addr ] = stack[i];
      }
      break;

    case kSetPwm_Op:     // Set pwm <enable>,<duty>,<freq>
      {
        uint8_t addr = kPwm_Enable_idx;
        uint8_t i = 0;
        for(; i<stack_idx; ++i,++addr)
          ctl_regs[ addr ] = stack[i];
        pwm1_update();
      }
      break;
      
    case kNoteOnVel_Op:   // Turn on note <vel>
      if( stack_idx == 1 )
      {
        uint8_t addr = stack[0] >> 2; // divide by 2 (we have only 64 entries in the table)
        
        ctl_regs[ kTmr_Coarse_idx ] = table[ addr ];
        ctl_regs[ kTmr_Fine_idx ]   = table[ addr+1 ];
      }
      tmr0_reset();
      break;
      
    case kNoteOnUsec_Op:  // Turn on note <coarse> <fine>      
      if( stack_idx == 2 )
      {
        ctl_regs[ kTmr_Coarse_idx ] = stack[0];
        ctl_regs[ kTmr_Fine_idx ]   = stack[1];
      }
      tmr0_reset();
      break;
      
    case kNoteOff_Op:     // Turn off note
      PORTB &= ~(_BV(ATTK_PIN) + _BV(HOLD_PIN));
      break;

    case kRead_Op: // Read a value {{ <src>} <addr> }
      if( stack_idx > 0)
      {
        ctl_regs[ kRead_Src_idx ] = stack[0];
      }
      
      if( stack_idx > 1 )
      {
        uint8_t reg_addr = 4;
        
        switch( ctl_regs[ kRead_Src_idx ] )
        {
          case 0: reg_addr = kReg_Addr_idx;   break;
          case 1: reg_addr = kTable_Addr_idx; break;
          case 2: reg_addr = kEE_Addr_idx;    break;
          default:
            // TODO: signal error
            break;
        }

        if( reg_addr <= 2 )
          ctl_regs[ reg_addr ] = stack[1];

        
      }
  } 
}



int main(void)
{
  cli();        // mask all interupts


  DDRB  |=   _BV(ATTK_DIR) + _BV(HOLD_DIR) + _BV(LED_DIR);  // setup PB4,PB3,PB1 as output  
  PORTB &= ~(_BV(ATTK_PIN) + _BV(HOLD_PIN) + _BV(LED_PIN)); // clear output pins

  
  timer0_init();
  pwm1_init();
  
  // setup i2c library
  usi_onReceiverPtr = on_receive; 
  usi_onRequestPtr  = on_request;
  usiTwiSlaveInit(I2C_SLAVE_ADDRESS);
  
  sei();

  PINB = _BV(LED_PIN);  // writes to PINB toggle the pins
  _delay_ms(1000);  
  PINB = _BV(LED_PIN);  // writes to PINB toggle the pins

  
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



