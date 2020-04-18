
//if you do not want read buffer, comment out next line
#define PICOSERIAL_USE_READBUFF

//if you do not want write buffer, comment out next line
#define PICOSERIAL_USE_WRITEBUFF

//you can set read/write buffer size at once
#define PICOSERIAL_BUFF_SIZE      8

//or separately.
#define PICOSERIAL_READBUFF_SIZE      2
#define PICOSERIAL_WRITEBUFF_SIZE     16

//PICOSERIAL_BUFF_SIZE is act like a default value of read/write buffer size

//if you want to use own read isr, do not define PICOSERIAL_USE_READBUFF
//define PICOSERIAL_ISR_READFUNC(c) and PICOSERIAL_CB_READFUNC()

// Gets called when a new bytes has arrived over serial
//#define PICOSERIAL_ISR_READFUNC(c)  rxfunc(c)

// Gets called when read() method invoked
//#define PICOSERIAL_CB_READFUNC()    cbfunc()

#include "PicoSerial.h"


void setup() {
  // put your setup code here, to run once:
  PicoSerial.begin( 9600 );
}

void loop() {
  // put your main code here, to run repeatedly:
  int c = PicoSerial.read();

  if( c != -1 ){
    PicoSerial.write( c );
  }
}
