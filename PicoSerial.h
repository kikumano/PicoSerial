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

#if defined(UBRR0H) || defined(DOXYGEN)

#if defined(UBRR0H)
//#warning UBRR0H -> UBRRnH
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
#endif

const uint16_t MIN_2X_BAUD = F_CPU/(4*(2*0XFFF + 1)) + 1;

// Gets called when a new bytes has arrived over serial
//#define PICOSERIAL_ISR_READFUNC(c)  rxfunc(c)

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
  }

  uint32_t calc_realbaudrate(uint32_t baud) {
    const bool f = ((F_CPU != 16000000UL || baud != 57600) && baud > MIN_2X_BAUD);
    const uint32_t ret = (F_CPU / (f? 4: 8) / baud - 1) / 2;

    return F_CPU/(8*(ret+1)); // Tell caller the BAUD they really got
  }

/*
=pod

  int read()			// Unbuffered read.  returns -1 if no character is available or an available character. Do not use if you supplied an rxCallBackFunction prior
=cut
*/
  int read() {
    return bitRead(UCSRnA, RXCn)? (unsigned int)UDRn: -1;
  } // read

/*
=pod

  size_t write(uint8_t b)	// Unbuffered write; param[in] b byte to write. return 1
=cut
*/
  size_t write(uint8_t b) {
    while (bitRead(UCSRnB, UDRIEn) || !bitRead(UCSRnA, UDREn)) {}
    UDRn = b;
    return 1;
  } // write

/*
=pod

  boolean canWrite();		// true if we can do a nonblocking write next
=cut
*/
  boolean canWrite() {
    return !(bitRead(UCSRnB, UDRIEn) || !bitRead(UCSRnA, UDREn));
  } // write

} PicoSerial; // PicoSerial

ISR(USART_RX_vect) {
  int c=UDRn; // must read, to clear the interrupt flag
#if defined(PICOSERIAL_ISR_READFUNC)
  PICOSERIAL_ISR_READFUNC(c);
#endif
}

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
