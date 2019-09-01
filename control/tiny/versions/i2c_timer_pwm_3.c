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
 kSetPwm_Op         =  0,  // Set PWM duty/hz/div   0 {<duty> {<freq> {<div>}}}  div:2=2,3=4,4=8,5=16,6=32,7=64,8=128,9=256,10=512,11=1024,12=2048,13=4096,14=8192,15=16384
 kNoteOnVel_Op      =  1,  // Turn on note          3 {<vel>}
 kNoteOnUsec_Op     =  2,  // Turn on note          4 {<coarse> {<fine> {<prescale>}}}
 kNoteOff_Op        =  3,  // Turn off note         5
 kSetReadAddr_Op    =  4,  // Set a read addr.      6 {<src>} {<addr>} }  src: 0=reg 1=table 2=eeprom
 kWrite_Op          =  5,  // Set write             7 {<addrfl|src> {addr}  {<value0> ... {<valueN>}}  addrFl:0x80  src: 4=reg 5=table 6=eeprom
 kSetMode_Op        =  6,  // Set the mode flags    8 {<mode>}  1=repeat 2=pwm
 kWriteTable_Op     =  7,  // Write table to EEprom 9 
 kInvalid_Op        =  8   //                                             
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
 kTmr_Prescale_idx   = 10,  // Timer 0 clock divider: 1=1,2=8,3=64,4=256,5=1024  Default: 4 (16us)
 
 kPwm_Duty_idx       = 11,  // 
 kPwm_Freq_idx       = 12,  //
 kPwm_Div_idx        = 13,  //

 kMode_idx           = 14, // 1=repeat 2=pwm
 kState_idx          = 15, // 1=attk 2=hold
 kError_Code_idx     = 16, // Error Code
 kMax_idx
};

enum
{
 kMode_Repeat_Fl = 1,
 kMode_Pwm_Fl    = 2,
 kState_Attk_Fl        = 1,
 kState_Hold_Fl        = 2
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
 254,                // 12 (0-255)  Pwm Frequency  (123 Hz)
  10,                // 13 (0-15)   Pwm clock div 
   
   kMode_Repeat_Fl,  // 14 mode flags  1=Repeat 2=PWM
   0,                // 15 state flags 1=attk   2=hold  (read/only)
   0,                // 16 (0-255)  Error bit field    
};

// These registers are saved to Eeprom
uint8_t eeprom_addr[] =
{
 kTmr_Prescale_idx,
 kPwm_Duty_idx,
 kPwm_Freq_idx,
 kPwm_Div_idx
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

void write_table()
{
  uint8_t i;
  uint8_t regN = sizeof(eeprom_addr);

  // write the persistent registers
  for(i=0; i<regN; ++i)
    EEPROM_write( i, ctl_regs[ eeprom_addr[i] ] );

  // write the table
  for(i=0; i<tableN; ++i)
    EEPROM_write( regN+i, table[i] );
}

void load_table()
{
  uint8_t i;
  uint8_t regN = sizeof(eeprom_addr);

  // read the persistent registers
  for(i=0; i<regN; ++i)
    ctl_regs[ eeprom_addr[i] ] = EEPROM_read(i);

  // read the tabke
  for(i=0; i<tableN; ++i)
    table[i] = EEPROM_read(regN + i);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Timer0
//

volatile uint8_t tmr0_state        = 0;    // current timer mode: 0=disabled 1=coarse mode, 2=fine mode 
volatile uint8_t tmr0_coarse_cur   = 0;

#define set_attack()    do { ctl_regs[kState_idx] |= kState_Attk_Fl;  PORTB |= _BV(ATTK_PIN);            } while(0)
#define clear_attack()  do { PORTB &= ~_BV(ATTK_PIN);           ctl_regs[kState_idx] &= ~kState_Attk_Fl; } while(0)


// Use the current tmr0 ctl_reg[] values to set the timer to the starting state.
void tmr0_reset()
{
  tmr0_coarse_cur       = 0;               // clear the coarse time counter
  ctl_regs[kState_idx] |= kState_Attk_Fl;  // set the attack state
  PORTB                |= _BV(ATTK_PIN);   // set the attack pin 
  
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
  
  TIMSK |= _BV(OCIE0A);     // enable the timer interrupt
}

ISR(TIMER0_COMPA_vect)
{
  switch( tmr0_state )
  {
    case 0:
      // timer is disabled
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

      // This marks the end of a timer period 

      // If in repeat mode ...
      if(ctl_regs[kMode_idx] & kMode_Repeat_Fl)
      {
        // store the current state of the attack flag
        uint8_t fl = ctl_regs[kState_idx] & kState_Attk_Fl;
        
        tmr0_reset();  // ... restart the timer
        
        // ATTK_PIN is always set after tmr0_reset() but we need to toggle in 'repeat' mode
        if( fl )
        {
          clear_attack();
        }

        // In repeat mode we always run the PWM output continuously.
        // This guarantees that no matter how the modes may be changing that PWM will be enabled.
        TIMSK  |= _BV(OCIE1B) + _BV(TOIE1); // Enable PWM interrupts

      }
      else  // ... not in repeat mode
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
  TIMSK  &= ~_BV(OCIE0A);                 // Disable interrupt TIMER1_OVF
  TCCR0A  |=  0x02;                       // CTC mode
  TCCR0B  |= ctl_regs[kTmr_Prescale_idx]; // set the prescaler
  GTCCR   |= _BV(PSR0);                   // Set the pre-scaler to the selected value
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
  TCCR1  |= ctl_regs[ kPwm_Div_idx];    // 32us period (512 divider) prescaler
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
      for(i=0; i<stack_idx && i<3; ++i)
        ctl_regs[ kPwm_Duty_idx + i ] = stack[i];

      // if the PWM prescaler was changed
      if( i == 3 )
        pwm1_init();
          
      pwm1_update();
      break;

      
    case kNoteOnUsec_Op:
      for(i=0; i<stack_idx && i<3; ++i)
        ctl_regs[ kTmr_Coarse_idx + i ] = stack[i];

      // if a prescaler was included then the timer needs to be re-initialized
      if( i == 3 )
        tmr0_init();
      
      tmr0_reset();
      break;

    case kNoteOff_Op:
      TIMSK  &= ~_BV(OCIE0A);                // clear timer interrupt (shouldn't be necessary)
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

        // if repeat mode was enabled
        if( ctl_regs[ kMode_idx ] & kMode_Repeat_Fl )
          tmr0_reset();
        
        pwm1_init();  // the state of PWM may have been changed
      }
      break;

    case kWriteTable_Op:
      write_table();
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



