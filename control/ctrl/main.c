// Note this program is designed to pair with c_client.
// The i2c connection may be made directly between the two Arduino SDA and SCL pins.
// No i2c pullup resistors are require.d
#define F_CPU 16000000UL
#define BAUD 38400

#include <util/setbaud.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "twi.h"

//------------------------------------------------------------------------------

#define SER_BUF_N (16)  // size of receive buffer

// Note that 'ser_buf_i_idx' must be declared volatile or the
// the compare in the main loop will not work.
volatile int  ser_buf_i_idx = 0;  // receive buffer input index
int           ser_buf_o_idx = 0;  // receive buffer output index

// Receive buffer
char buf[ SER_BUF_N ];


void uart_init(void)
{
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8-bit data 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);   // Enable RX and TX and RX intertupt enable
}

void uart_putchar(char c)
{
  loop_until_bit_is_set(UCSR0A, UDRE0); // Wait until data register empty.
  UDR0 = c;
}

void uart_putchar_alt(char c)
{
  UDR0 = c;
  loop_until_bit_is_set(UCSR0A, TXC0); // Wait until transmission ready. 
}

char uart_getchar(void)
{
  loop_until_bit_is_set(UCSR0A, RXC0); // Wait until data exists. 
  return UDR0;
}

//------------------------------------------------------------------------------

#define I2C_LOCAL_ADDR  (9)
#define I2C_REMOTE_ADDR (8)

#define I2C_BUF_N (16)
static          uint8_t i2c_buf[ I2C_BUF_N ]; 
static volatile uint8_t i2c_buf_i_idx = 0;
static          uint8_t i2c_buf_o_idx = 0;

static          uint8_t last_char = '0';

void i2c_read_from( uint8_t i2c_addr, uint8_t dev_reg_addr, uint8_t read_byte_cnt )
{
        uint8_t recv_char     = '0';  
  const uint8_t kWaitFl       = 1;
  const uint8_t kSendStopFl   = 1;
  const uint8_t kNoSendStopFl = 0;

  // Request to read from the client. Note that 'sendStop'==0.
  // Use this call to tell the client what data should be sent
  // during the subsequent twi_readFrom().
  twi_writeTo(I2C_REMOTE_ADDR, &dev_reg_addr, 1, kWaitFl, kNoSendStopFl);

      
  // Blocking waiting and wait to read the client's response.
  for( uint8_t i=0; i<read_byte_cnt; ++i)
    if( twi_readFrom(I2C_REMOTE_ADDR, &recv_char, 1, i==read_byte_cnt-1) )
      uart_putchar(recv_char);

  PORTB ^= _BV(PORTB5);   //  toggle LED
 
  
}

uint8_t i2c_xmit( uint8_t remote_addr, uint8_t* buf, uint8_t n, uint8_t sendStopFl)
{
  return twi_writeTo(remote_addr, buf, n, 1, sendStopFl);
}

void i2c_init()
{
  twi_setAddress(I2C_LOCAL_ADDR);
  twi_init();
}

ISR(USART_RX_vect)
{
  // receive the incoming byte
  buf[ ser_buf_i_idx ] = uart_getchar();

  // advance the buffer input index
  ser_buf_i_idx = (ser_buf_i_idx + 1) % SER_BUF_N;  
}


//--------------------------------------------------------------------------------------------------
static volatile int timerFl = 0;
  

ISR(TIMER1_COMPA_vect)
{
  timerFl = 1;
}

void timer_init(void)
{
  TCCR1A = 0;              // set timer control registers to default
  TCCR1B = 0;              // 
  OCR1A = 15624;           // 1 Hz = 16Mghz / (1024 * 15624)
  TCCR1B |= (1 << WGM12);  // CTC mode on
  TCCR1B |= (1 << CS10);   // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS12);   // 
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt  
}

//--------------------------------------------------------------------------------------------------


int main (void)
{
  enum { kWait_for_cmd, kWait_for_i2c, kWait_for_reg, kWait_for_cnt, kWait_for_value };
  const uint8_t kWaitFl       = 1;
  const uint8_t kSendStopFl   = 1;
  const uint8_t kNoSendStopFl = 0;
  
  char    c;
  uint8_t state = kWait_for_cmd;
  char    cmd;
  uint8_t i2c_addr;
  uint8_t dev_reg_addr;
  uint8_t op_byte_cnt;
  
  cli();        // mask all interupts

  DDRB |= _BV(DDB5);  // set led pin for output
  
  timer_init(); // setup the timer
  uart_init();  // setup UART data format and baud rate
  i2c_init();
  sei();        // re-enable interrupts

  uart_putchar('a');
  
  for(;;)
  {
    if( timerFl )
    {
      //PORTB ^= _BV(PORTB5);   //  toggle LED
      
      timerFl = 0;
    }
    
    // if there are bytes waiting in the serial buffer
    if( ser_buf_o_idx != ser_buf_i_idx )
    {
      // get the waiting byte
      c = buf[ser_buf_o_idx];

      // advance the buffer output index
      ser_buf_o_idx = (ser_buf_o_idx+1) % SER_BUF_N;

      //  Serial Protocol
      //  'r', reg-idx, cnt,                      -> i2c_read_from()
      //  'w', reg-idx, cnt, value0, ... valueN   -> i2c_xmit()
      
      switch(state)
      {
        case kWait_for_cmd:
          if(c == 'w' || c == 'r')
          {  
            cmd   = c;
            state = kWait_for_i2c;
          }
          else
            uart_putchar('E');
          break;

        case kWait_for_i2c:
          i2c_addr = (uint8_t)c;
          state = kWait_for_reg;
          break;
          
        case kWait_for_reg:
          dev_reg_addr = (uint8_t)c;
          state = kWait_for_cnt;
          break;

        case kWait_for_cnt:
          op_byte_cnt = (uint8_t)c;
          
          if( cmd == 'r' )
          {
            i2c_read_from( i2c_addr, dev_reg_addr, op_byte_cnt );
            state = kWait_for_cmd;
          }
          else
          {
            state = kWait_for_value;
          }
          break;
            
        case kWait_for_value:
          {
            uint8_t buf[] = { dev_reg_addr, c };
                      
            i2c_xmit( I2C_REMOTE_ADDR, buf, 2, kSendStopFl);
            state = kWait_for_cmd;
          }
          break;
            
      }            
    }        
  }    
}




