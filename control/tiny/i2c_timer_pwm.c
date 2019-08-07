// w 60 0  1 10    : w i2c_addr SetPWM enable duty_val
// w 60 5 12  8 32 : w i2c_addr write addrFl|src coarse_val
// w 60 4  0  5    : w i2c_addr read  src read_addr  (set the read address to register 5)
// r 60 4  3       : r i2c_addr <dum> cnt            (read the first 3 reg's beginning w/ 5)
/*                                    
                                    AT TINY 85
                                     +--\/--+
                              RESET _| 1  8 |_ +5V
             ~OC1B       HOLD  DDB3 _| 2  7 |_ SCL         yellow 
              OC1B      ONSET  DDB4 _| 3  6 |_ DDB1 LED
                                GND _| 4  5 |_ SDA         orange
                                     +------+
        * = Serial and/or programming pins on Arduino as ISP
*/


// This program acts as the device (slave) for the control program i2c/a2a/c_ctl
#define F_CPU 16000000L

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
 kSetPwm_Op         =  0,  // Set PWM registers  0 {<duty> {<freq>}}
 kNoteOnVel_Op      =  1,  // Turn on note       1 {<vel>}
 kNoteOnUsec_Op     =  2,  // Turn on note       2 {<coarse> {<fine> {<prescale>}}}
 kNoteOff_Op        =  3,  // Turn off note      3
 kSetReadAddr_Op    =  4,  // Set a read addr.   4 {<src>} {<addr>} }  src: 0=reg 1=table 2=eeprom
 kWrite_Op          =  5,  // Set write          5 {<addrfl|src> {addr}  {<value0> ... {<valueN>}}  addrFl:0x80  src: 4=reg 5=table 6=eeprom
 kSetMode_Op        =  6,  // Set the mode flags 6 {<mode>}  1=repeat 2=pwm
 
 kInvalid_Op        =  7   //                                             
};


enum
{
 kReg_Rd_Addr_idx    =  0,  // Next Reg Address to read
 kTable_Rd_Addr_idx  =  1,  // Next Table Address to read
 kEE_Rd_Addr_idx     =  2,  // Next EEPROM address to read
 kRead_Src_idx       =  3,  // kReg_Rd_Addr_idx=reg,  kTable_Rd_Addr_idx=table, kEE_Rd_Addr_idx=eeprom
 
 kReg_Wr_Addr_idx    =  4,  // Next Reg Address to write
 kTable_Wr_Addr_idx  =  5,  // Next Table Address to write
 kEE_Wr_Addr_idx     =  6,  // Next EEPROM address to write
 kWrite_Dst_idx      =  7,  // kReg_Wr_Addr_idx=reg,  kTable_Wr_Addr_idx=table, kEE_Wr_Addr_idx=eeprom
 
 kTmr_Coarse_idx     =  8,  //  
 kTmr_Fine_idx       =  9,  // 
 kTmr_Prescale_idx   = 10,  // Timer 0 clock divider: 1=1,2=8,3=64,4=256,5=1024  Default: 8 (16us)
 
 kPwm_Duty_idx       = 11,  // 
 kPwm_Freq_idx       = 12,  //

 kMode_idx           = 13, // 1=repeat 2=pwm
 kState_idx          = 14, // 1=attk 2=hold
 kError_Code_idx     = 15, // Error Code
 kMax_idx
};

enum
{
 kMode_Repeat_Fl = 1,
 kMode_Pwm_Fl    = 2,
 kAttk_Fl        = 1,
 kHold_Fl        = 2
};


#define isInRepeatMode() ctl_regs[ kMode_idx ] & kMode_Repeat_Fl
#define isInPwmMode()    ctl_regs[ kMode_idx ] & kMode_Pwm_Fl

// Flags:
// 1=Repeat: 1=Timer and PWM are free running. This allows testing with LED's. 0=Timer triggers does not reset on time out. 
// 2=PWM:  On timer timeout  1=PWM HOLD 0=Set HOLD 

volatile uint8_t ctl_regs[] =
{
   0,                //  0 (0-(kMax_idx-1)) Reg Read Addr   
   0,                //  1 (0-255)          Table Read Addr
   0,                //  2 (0-255)          EE Read Addr  
   kReg_Rd_Addr_idx, //  3 (0-2)    Read source   
   0,                //  4 (0-(kMax_idx-1)) Reg Write Addr   
   0,                //  5 (0-255)          Table Write Addr
   0,                //  6 (0-255)          EE Write Addr     
   kReg_Wr_Addr_idx, //  7 (0-2)    Write source
 245,                //  8 (0-255)  Timer 0 Coarse Value 
  25,                //  9 (0-255)  Timer 0 Fine Value
   4,                // 10 (1-5)    4=16us per tick
 127,                // 11 (0-255)  Pwm Duty cycle
 254,                // 12 (0-255)  Pwm Frequency  (123 hz)
   kMode_Repeat_Fl,  // 13 mode flags  1=Repeat 2=PWM
   0,                // 14 state flags 1=attk   2=hold
   0,                // 15 (0-255)  Error bit field
};

#define tableN 256
uint8_t table[ tableN ]; // [ coarse_0,fine_0, coarse_1, fine_1, .... coarse_127,fine_127]
 

enum
{
 kInvalid_Read_Src_ErrFl  = 0x01,
 kInvalid_Write_Dst_ErrFl = 0x02
};

#define set_error( flag ) ctl_regs[ kError_Code_idx ] |= (flag)

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

/*
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
*/


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Timer0
//

volatile uint8_t tmr0_state        = 0;    // 0=disabled 1=coarse mode, 2=fine mode 
volatile uint8_t tmr0_coarse_cur   = 0;

#define set_attack()    do { ctl_regs[kState_idx] |= kAttk_Fl;  PORTB |= _BV(ATTK_PIN);            } while(0)
#define clear_attack()  do { PORTB &= ~_BV(ATTK_PIN);           ctl_regs[kState_idx] &= ~kAttk_Fl; } while(0)


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

  ctl_regs[kState_idx] |= kAttk_Fl;      // set the attack state
  PORTB                |= _BV(ATTK_PIN); // set the attack pin 
  TIMSK                |= _BV(OCIE0A);   // enable the timer interrupt
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

      // If in repeat mode
      if(ctl_regs[kMode_idx] & kMode_Repeat_Fl)
      {
        uint8_t fl = ctl_regs[kState_idx] & kAttk_Fl;
        
        tmr0_reset();  // restart the timer
        
        // ATTK_PIN is always set after tmr0_reset() but we need to toggle in 'repeat' mode
        if( fl )
        {
          clear_attack();
        }

        // In repeat mode we run the PWM output continuously
        TIMSK  |= _BV(OCIE1B) + _BV(TOIE1); // Enable PWM interrupts

      }
      else  // not in repeat mode
      {
        clear_attack();
        
        if( ctl_regs[kMode_idx] & kMode_Pwm_Fl)
        {
          TIMSK  |= _BV(OCIE1B) + _BV(TOIE1);    // PWM interupt Enable interrupts          
        }
        else
        {
          PORTB |= _BV(HOLD_PIN); // set the HOLD pin          
        }

        TIMSK &= ~_BV(OCIE0A);   // clear timer interrupt
        
      }
      
      break;
  }
}


void tmr0_init()
{
  TIMSK  &= ~_BV(OCIE0A);    // Disable interrupt TIMER1_OVF
  TCCR0A  |=  0x02;           // CTC mode
  TCCR0B  |= ctl_regs[kTmr_Prescale_idx]; // set the prescaler

  GTCCR   |= _BV(PSR0);      // Set the pre-scaler to the selected value
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Pwm
//
// PWM is optimized to use pins OC1A ,~OC1A, OC1B, ~OC1B
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
  TCCR1  |= 10;            // 32us period (512 divider) prescaler
  GTCCR  |= _BV(PWM1B);    // Enable PWM B and disconnect output pins
  GTCCR  |= _BV(PSR1);     // Set the pre-scaler to the selected value

  pwm1_update();
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
  
  switch( ctl_regs[ kRead_Src_idx ] )
  {
    case kReg_Rd_Addr_idx:
      val = ctl_regs[ ctl_regs[kReg_Rd_Addr_idx] ];
      break;

    case kTable_Rd_Addr_idx:
      val = table[ ctl_regs[kTable_Rd_Addr_idx] ];
      break;

    case kEE_Rd_Addr_idx:
      val = EEPROM_read(ctl_regs[kEE_Rd_Addr_idx]);
      break;

    default:
      set_error( kInvalid_Read_Src_ErrFl );
      return;
  }
  
  usiTwiTransmitByte(val);

  ctl_regs[ ctl_regs[ kRead_Src_idx ]  ] += 1;

}


void _write_op( uint8_t* stack, uint8_t stackN )
{
  uint8_t stack_idx = 0;
  
  if( stackN > 0 )
  {
    uint8_t src     = stack[0] & 0x07;
    uint8_t addr_fl = stack[0] & 0x08;

    // verify the source value
    if( src < kReg_Wr_Addr_idx  || src > kEE_Wr_Addr_idx )
    {
      set_error( kInvalid_Write_Dst_ErrFl );
      return;
    }

    // set the write source
    stack_idx                  = 1;
    ctl_regs[ kWrite_Dst_idx ] = src;

    // if an address value was passed also ....
    if( addr_fl && stackN > 1 )
    {
      stack_idx       = 2;
      ctl_regs[ src ] = stack[1];
    }
  }

  //
  for(; stack_idx<stackN; ++stack_idx)
  {
    uint8_t addr_idx = ctl_regs[ ctl_regs[kWrite_Dst_idx] ]++;
    uint8_t val      = stack[ stack_idx ];
    
    switch( ctl_regs[ kWrite_Dst_idx ] )
    {
      case kReg_Wr_Addr_idx:   ctl_regs[ addr_idx ]           = val;  break;
      case kTable_Wr_Addr_idx: table[ addr_idx ]              = val;  break;
      case kEE_Wr_Addr_idx:    EEPROM_write( table[ addr_idx ], val); break;

      default:
        set_error( kInvalid_Write_Dst_ErrFl );
        break;
    }
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
  PINB = _BV(LED_PIN);  // writes to PINB toggle the pins

  const uint8_t stackN = 16;
  uint8_t stack_idx = 0;
  uint8_t stack[ stackN ];
  uint8_t i;
  
  if (byteN < 1 || byteN > TWI_RX_BUFFER_SIZE)
  {
    // Sanity-check
    return;
  }

  // get the register index to read/write
  uint8_t op_id = usiTwiReceiveByte();
    
  byteN--;

  // If only one byte was received then this was a read request
  // and the buffer pointer (reg_position) is now set to return the byte
  // at this location on the subsequent call to on_request() ...
  if(byteN)
  {
    while( byteN-- )
    {  
      stack[stack_idx] = usiTwiReceiveByte();
      ++stack_idx;
    }
  }
  
  switch( op_id )
  {
    case kSetPwm_Op:
      for(i=0; i<stack_idx && i<2; ++i)
        ctl_regs[ kPwm_Duty_idx + i ] = stack[i];
      pwm1_update();
      break;
      
    case kNoteOnUsec_Op:
      for(i=0; i<stack_idx && i<3; ++i)
        ctl_regs[ kTmr_Coarse_idx + i ] = stack[i];
      tmr0_reset();
      break;

    case kNoteOff_Op:
      TIMSK  &= ~(_BV(OCIE1B) + _BV(TOIE1)); // PWM interupt disable interrupts          
      PORTB  &= ~_BV(HOLD_PIN);              // clear the HOLD pin          
      break;

    case kSetReadAddr_Op:
      if( stack_idx > 0 )
      {
        ctl_regs[ kRead_Src_idx ] = stack[0];
      
        if( stack_idx > 1 )
          ctl_regs[ ctl_regs[ kRead_Src_idx ] ] = stack[1];
      }
      break;

    case kWrite_Op:
      _write_op( stack, stack_idx );
      break;

    case kSetMode_Op:
      if( stack_idx > 0)
      {
        ctl_regs[ kMode_idx ] = stack[0];
        tmr0_reset();
      }
      
      break;
  }
}


int main(void)
{
  cli();        // mask all interupts

  DDRB  |=   _BV(ATTK_DIR)  + _BV(HOLD_DIR)  + _BV(LED_DIR);  // setup PB4,PB3,PB1 as output  
  PORTB &= ~(_BV(ATTK_PIN)  + _BV(HOLD_PIN)  + _BV(LED_PIN)); // clear output pins
  
  tmr0_init();
  pwm1_init();
  
  // setup i2c library
  usi_onReceiverPtr = on_receive; 
  usi_onRequestPtr  = on_request;
  usiTwiSlaveInit(I2C_SLAVE_ADDRESS);
  
  sei();

  PINB = _BV(LED_PIN);  // writes to PINB toggle the pins
  _delay_ms(1000);  
  PINB = _BV(LED_PIN);  // writes to PINB toggle the pins

  // if in repeat mode
  if( ctl_regs[ kMode_idx ] & kMode_Repeat_Fl)
    tmr0_reset();
  
  while(1)
  {

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



