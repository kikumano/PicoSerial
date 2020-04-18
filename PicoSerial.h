#ifndef PicoSerial_h
#define PicoSerial_h
#include <Arduino.h>

/*
 * cnd - see https://github.com/arduino/Arduino/wiki/Arduino-IDE-1.5:-Library-specification
 *
=head1 PicoSerial v1.0

Ultra lightweight serial support for Arduino, properly supporting incoming interrupts

=head2 SYNOPSIS

=for markdown
```C
  #include <PicoSerial.h>

  #define BAUD 250000
  uint32_t setup() {
    gotBAUD=PicoSerial.begin(BAUD,myIn);		// myIn is your code which gets called when bytes arrive
    PicoSerial.print("The BAUD I really got was:");
    PicoSerial.println(gotBAUD);			// If you ask for 115200 on an 8mhz CPU, you really get 111111 baud...
  }

  void loop() {
    // do stuff - usually checking your read buffer here...
  }

  void myIn(int c) {				// this is an ISR - it gets called when new data comes in. buffer and exit this as fast as you can; do not do anything else in here.
    myBuffer[bufferIndex++]=c;
  }
```

=cut

Convert all POD herein to markdown thusly:-

perl -MPod::Markdown -e 'Pod::Markdown->new->filter(@ARGV)' PicoSerial.h > README.md

*/

#if ( defined(UBRRH) || defined(UBRR0H) || defined(LINBRRH) \
  || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H) || defined(DOXYGEN) ) && !DISABLE_UART

#if defined(PICOSERIAL_USE_HWSERIAL3)
  #ifdef USART3_RX_vect
//  #warning USART3_RX_vect -> USARTn_RX_vect
  #define USARTn_RX_vect   USART3_RX_vect
  #define USARTn_UDRE_vect USART3_UDRE_vect
  #else
  #error "no fourth serial"
  #endif
  
  #if defined(UBRR3H)
//  #warning UBRR3H -> UBRRnH
  #define UBRRnH    UBRR3H
  #define UBRRnL    UBRR3L
  #define UCSRnA    UCSR3A
    #define RXCn      RXC3
    #define UDREn     UDRE3
    #define U2Xn      U2X3
  #define UCSRnB    UCSR3B
    #define RXCIEn    RXCIE3
    #define UDRIEn    UDRIE3
    #define RXENn     RXEN3
    #define TXENn     TXEN3
  #define UDRn      UDR3
  #endif
#elif defined(PICOSERIAL_USE_HWSERIAL2)
  #ifdef USART2_RX_vect
//  #warning USART2_RX_vect -> USARTn_RX_vect
  #define USARTn_RX_vect   USART2_RX_vect
  #define USARTn_UDRE_vect USART2_UDRE_vect
  #else
  #error "no third serial"
  #endif
  
  #if defined(UBRR2H)
//  #warning UBRR2H -> UBRRnH
  #define UBRRnH    UBRR2H
  #define UBRRnL    UBRR2L
  #define UCSRnA    UCSR2A
    #define RXCn      RXC2
    #define UDREn     UDRE2
    #define U2Xn      U2X2
  #define UCSRnB    UCSR2B
    #define RXCIEn    RXCIE2
    #define UDRIEn    UDRIE2
    #define RXENn     RXEN2
    #define TXENn     TXEN2
  #define UDRn      UDR2
  #endif
#elif defined(PICOSERIAL_USE_HWSERIAL1)
  #ifdef USART1_RX_vect
//  #warning USART1_RX_vect -> USARTn_RX_vect
  #define USARTn_RX_vect   USART1_RX_vect
  #define USARTn_UDRE_vect USART1_UDRE_vect
  #elif defined(USART1_RXC_vect)
//  #warning USART1_RXC_vect -> USARTn_RX_vect
  #define USARTn_RX_vect   USART1_RXC_vect
  #define USARTn_UDRE_vect USART1_UDRE_vect
  #else
  #error "no second serial"
  #endif
  
  #if defined(UBRR1H)
//  #warning UBRR1H -> UBRRnH
  #define UBRRnH    UBRR1H
  #define UBRRnL    UBRR1L
  #define UCSRnA    UCSR1A
    #define RXCn      RXC1
    #define UDREn     UDRE1
    #define U2Xn      U2X1
  #define UCSRnB    UCSR1B
    #define RXCIEn    RXCIE1
    #define UDRIEn    UDRIE1
    #define RXENn     RXEN1
    #define TXENn     TXEN1
  #define UDRn      UDR1
  #endif
#else
  #if defined(USART_RX_vect)
//  #warning USART_RX_vect -> USARTn_RX_vect
  #define USARTn_RX_vect   USART_RX_vect
  #define USARTn_UDRE_vect USART_UDRE_vect
  #elif defined(USART0_RX_vect)
//  #warning USART0_RX_vect -> USARTn_RX_vect
  #define USARTn_RX_vect   USART0_RX_vect
  #define USARTn_UDRE_vect USART0_UDRE_vect
  #elif defined(UART_RX_vect)
//  #warning UART_RX_vect -> USARTn_RX_vect
  #define USARTn_RX_vect   UART_RX_vect
  #define USARTn_UDRE_vect UART_UDRE_vect
  #elif defined(UART0_RX_vect)
//  #warning UART0_RX_vect -> USARTn_RX_vect
  #define USARTn_RX_vect   UART0_RX_vect
  #define USARTn_UDRE_vect UART0_UDRE_vect
  #elif defined(LIN_TC_vect)
//  #warning LIN_TC_vect -> USARTn_RX_vect
// do nothing
  #else
  #error "no serial"
  #endif

  #if defined(UBRRH)
//  #warning UBRRH -> UBRRnH
  #define UBRRnH    UBRRH
  #define UBRRnL    UBRRL
  #define UCSRnA    UCSRA
    #define RXCn      RXC
    #define UDREn     UDRE
    #define U2Xn      U2X
  #define UCSRnB    UCSRB
    #define RXCIEn    RXCIE
    #define UDRIEn    UDRIE
    #define RXENn     RXEN
    #define TXENn     TXEN
  #define UDRn      UDR
  #elif defined(UBRR0H)
//  #warning UBRR0H -> UBRRnH
  #define UBRRnH    UBRR0H
  #define UBRRnL    UBRR0L
  #define UCSRnA    UCSR0A
    #define RXCn      RXC0
    #define UDREn     UDRE0
    #define U2Xn      U2X0
  #define UCSRnB    UCSR0B
    #define RXCIEn    RXCIE0
    #define UDRIEn    UDRIE0
    #define RXENn     RXEN0
    #define TXENn     TXEN0
  #define UDRn      UDR0
  #elif defined(LINDAT)
//  #warning LINDAT -> UDRn
  #define UCSRnA    LINSIR
    #define RXCn      LRXOK
    #define UDREn     LTXOK
    #define U2Xn      // n/a
  #define UCSRnB    LINENIR
    #define RXCIEn    LENRXOK
    #define UDRIEn    LENTXOK
    #define RXENn     // n/a
    #define TXENn     // n/a
  #define UDRn      LINDAT
  #endif
#endif


#ifndef PICOSERIAL_BUFF_SIZE
//max 0x80(128) bytes
#define PICOSERIAL_BUFF_SIZE      8
#endif
#ifndef PICOSERIAL_READBUFF_SIZE
#define PICOSERIAL_READBUFF_SIZE      PICOSERIAL_BUFF_SIZE
#endif
#ifndef PICOSERIAL_WRITEBUFF_SIZE
#define PICOSERIAL_WRITEBUFF_SIZE     PICOSERIAL_BUFF_SIZE
#endif

#define PICOSERIAL_BUFFFULL_BIT   7
#if ( PICOSERIAL_READBUFF_SIZE > 0x80 ) || ( PICOSERIAL_WRITEBUFF_SIZE > 0x80 )
  #error "large buffer is not supported. must be 128 or less."
#endif

#ifdef PICOSERIAL_USE_READBUFF
volatile byte PicoSerial_buff_rx[PICOSERIAL_READBUFF_SIZE];
volatile byte buff_rx_in = 0;
volatile byte buff_rx_out = 0;
#endif
#ifdef PICOSERIAL_USE_WRITEBUFF
volatile byte PicoSerial_buff_tx[PICOSERIAL_WRITEBUFF_SIZE];
volatile byte buff_tx_in = 0;
volatile byte buff_tx_out = 0;
#endif


const uint16_t MIN_2X_BAUD = F_CPU/(4*(2*0XFFF + 1)) + 1;

// Gets called when a new bytes has arrived over serial
//#define PICOSERIAL_ISR_READFUNC(c)  rxfunc(c)

// Gets called when read() method invoked
//#define PICOSERIAL_CB_READFUNC()    cbfunc()

class PicoSerial : public Print {
 public:
  using Print::write;
/*
=head2 FUNCTIONS

=for markdown
  ```C

    uint32_t begin(BAUD,rxCallBackFunction);		// Sets baud rate, and lets you tell PicoSerial which of your functions you want to call when data is ready. Returns baudrate
=cut
*/
  void begin(uint32_t baud) {  // Do not call this function if you use another serial library.
#ifdef LINENIR
    LINCR = bit( LSWRES );
    LINBRR = ( ( ( F_CPU * 10L / 16L / baud ) + 5L ) / 10L ) - 1;
    LINBTR = bit( LDISR ) | ( 16 << LBT0 );
    LINCR = bit( LENA ) | bit( LCMD2 ) | bit( LCMD1 ) | bit( LCMD0 );
    bitSet( LINENIR, LENRXOK );
#else
    uint16_t baud_setting;
    noInterrupts();                      // disable all interrupts
    // don't worry, the compiler will squeeze out F_CPU != 16000000UL
    if ((F_CPU != 16000000UL || baud != 57600) && baud > MIN_2X_BAUD) {
      UCSRnA = bit(U2Xn); // Double the USART Transmission Speed
      baud_setting = (F_CPU / 4 / baud - 1) / 2;
    } else {
      // hardcoded exception for compatibility with the bootloader shipped
      // with the Duemilanove and previous boards and the firmware on the 8U2
      // on the Uno and Mega 2560.
      UCSRnA = 0;
      baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }
    // assign the baud_setting
    UBRRnH = highByte( baud_setting );
    UBRRnL = lowByte( baud_setting );
    // enable transmit and receive
    UCSRnB |= bit(TXENn) | bit(RXENn) | bit(RXCIEn);
    interrupts();                      // enable all interrupts
#endif
  }

  uint32_t calc_realbaudrate(uint32_t baud) {
#ifdef LINENIR
//xxx fixme
    return baud; // Tell caller the BAUD they really got
#else
    const bool f = ((F_CPU != 16000000UL || baud != 57600) && baud > MIN_2X_BAUD);
    const uint32_t ret = (F_CPU / (f? 4: 8) / baud - 1) / 2;

    return F_CPU/(8*(ret+1)); // Tell caller the BAUD they really got
#endif
  }

/*
=pod

  int read()			// Unbuffered read.  returns -1 if no character is available or an available character. Do not use if you supplied an rxCallBackFunction prior
=cut
*/
  int read() {
#ifdef PICOSERIAL_USE_READBUFF
  if( buff_rx_in == buff_rx_out ){
    return -1;
  }

  //disable rx intr
  bitClear( UCSRnB, RXCIEn );
  const byte re = PicoSerial_buff_rx[buff_rx_out++];
  buff_rx_out %= sizeof PicoSerial_buff_rx;
  bitClear( buff_rx_in, PICOSERIAL_BUFFFULL_BIT );
  //enable rx intr
  bitSet( UCSRnB, RXCIEn );

  return (unsigned int)re;
#elif defined(PICOSERIAL_ISR_READFUNC)
    return PICOSERIAL_CB_READFUNC();
#else
    return bitRead(UCSRnA, RXCn)? (unsigned int)UDRn: -1;
#endif
  } // read

/*
=pod

  size_t write(uint8_t b)	// Unbuffered write; param[in] b byte to write. return 1
=cut
*/
  size_t write(uint8_t b) {
#ifdef PICOSERIAL_USE_WRITEBUFF
  if( !bitRead( UCSRnB, UDRIEn ) ){
    UDRn = b;
    bitSet( UCSRnB, UDRIEn );
    return 1;
  }
  
  for( ; ; ){
    if( !bitRead( buff_tx_in, PICOSERIAL_BUFFFULL_BIT ) ){
      //disable tx intr
      bitClear( UCSRnB, UDRIEn );
      
      PicoSerial_buff_tx[buff_tx_in++] = b;
      buff_tx_in %= sizeof PicoSerial_buff_tx;
      if( buff_tx_in == buff_tx_out ){ bitSet( buff_tx_in, PICOSERIAL_BUFFFULL_BIT ); }
      
      //enable tx intr
      bitSet( UCSRnB, UDRIEn );
      break;
    }
  }
#else
    while (bitRead(UCSRnB, UDRIEn) || !bitRead(UCSRnA, UDREn)) {}
    UDRn = b;
#endif
    return 1;
  } // write

/*
=pod

  boolean canWrite();		// true if we can do a nonblocking write next
=cut
*/
  boolean canWrite() {
#ifdef PICOSERIAL_USE_WRITEBUFF
    return !bitRead( buff_tx_in, PICOSERIAL_BUFFFULL_BIT );
#else
    return !(bitRead(UCSRnB, UDRIEn) || !bitRead(UCSRnA, UDREn));
#endif
  } // write

} PicoSerial; // PicoSerial


#ifdef LIN_TC_vect
#if defined(PICOSERIAL_USE_READBUFF) || defined(PICOSERIAL_ISR_READFUNC) || defined(PICOSERIAL_USE_WRITEBUFF)
ISR(LIN_TC_vect) {
#endif
#endif // LIN_TC_vect

#if defined(PICOSERIAL_USE_READBUFF) || defined(PICOSERIAL_ISR_READFUNC)
#ifdef LIN_TC_vect
if( bitRead( LINSIR, LRXOK ) ){
#else
ISR(USARTn_RX_vect) {
#endif // LIN_TC_vect
  const byte c=UDRn; // must read, to clear the interrupt flag

#ifdef PICOSERIAL_USE_READBUFF
  if( !bitRead( buff_rx_in, PICOSERIAL_BUFFFULL_BIT ) ){
    PicoSerial_buff_rx[buff_rx_in++] = c;
    buff_rx_in %= sizeof PicoSerial_buff_rx;
    if( buff_rx_in == buff_rx_out ){ bitSet( buff_rx_in, PICOSERIAL_BUFFFULL_BIT ); }
//  } else {
    //buffer overflow
  }
#elif defined(PICOSERIAL_ISR_READFUNC)
  PICOSERIAL_ISR_READFUNC(c);
#endif
}
#endif // defined(PICOSERIAL_USE_READBUFF) || defined(PICOSERIAL_ISR_READFUNC)

#ifdef PICOSERIAL_USE_WRITEBUFF
#ifdef LIN_TC_vect
if( bitRead( LINSIR, LTXOK ) ){
#else
ISR(USARTn_UDRE_vect) {
#endif // LIN_TC_vect
  if( buff_tx_in == buff_tx_out ){
    //buffer empty
    //disable tx intr
    bitClear( UCSRnB, UDRIEn );
    return;
  }

  const byte b = PicoSerial_buff_tx[buff_tx_out++];
  buff_tx_out %= sizeof PicoSerial_buff_tx;
  bitClear( buff_tx_in, PICOSERIAL_BUFFFULL_BIT );
  
  UDRn = b;
}
#endif // PICOSERIAL_USE_WRITEBUFF

#ifdef LIN_TC_vect
#if defined(PICOSERIAL_USE_READBUFF) || defined(PICOSERIAL_ISR_READFUNC) || defined(PICOSERIAL_USE_WRITEBUFF)
}
#endif
#endif // LIN_TC_vect

#endif  // defined(UDRn) || defined(DOXYGEN)
#endif  // PicoSerial_h


/*
=pod

=for markdown
```

=head2 Thanks

This code was adapted from PetitSerial.h, which is a small Serial class in the spirit of Petit FatFs.

=head2 HOW TO INSTALL

=head3 (Method 1)

=for markdown
1. Open a terminal (commandline, bash shell, whatever)
2. Change into your Arduino folder
```bash
   cd /Applications/Arduino.app/Contents/Java/libraries/
   (or)
   cd ~/Arduino/libraries/
```
3. grab and install this code
```bash
   git clone https://github.com/gitcnd/PicoSerial.git
```
4. restart your arduino IDE
5. Choose File => Examples => PicoSerial => PicoSerial_example
6. Hit the "build" button and enjoy!

=head3 (Method 2) - see https://www.arduino.cc/en/Guide/Libraries

=for markdown
1. Download the ZIP of this repo: https://github.com/gitcnd/PicoSerial/archive/master.zip
2. In your IDE, select Sketch -> Include Library -> Add .ZIP Library
3. Choose File => Examples => PicoSerial => PicoSerial_example
4. Hit the "build" button and enjoy!

=cut
*/
