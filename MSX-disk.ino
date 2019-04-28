// read/write disk sectors (512 bytes)

#define SYSFREQ 16000000  // Main system frequency for Arduino Nano is 16MHz
#define BAUDRATE 1200  // required baudrate 1200,2400, ~3200 msx for MSX
#define SPACE (SYSFREQ/BAUDRATE/2) //6666 half period of '0' for 1200Hz @ 16MHz timer incr
#define MARK (SPACE/2) //3333 half period of '1' for 2400Hz @ 16MHz timer incr
#define HEADER 16

// global vars (disk sector buffer and command buffer)
byte sector[512]; // disk sector buffer
byte cmd[4];  //bytes  0: 'R'/'W' 1: disk_nr 2: sector_nr low byte 3: sector_nr high byte
// interrupt routine for tape output
volatile word intcnt,timerinc,pos,endpos;
volatile byte bitcnt,tape;
const byte motorIntPin = 3;
 
// MSX 'bsave' file format; this is the driver for MSXvDisk
const byte cas[] PROGMEM=
{ 0xD0,0xD0,0xD0,0xD0,0xD0,0xD0,0xD0,0xD0,0xD0,0xD0,0x6D,0x73,0x78,0x64,0x73,0x6B,
0x00,0xD0,0x7F,0xD1,0x00,0xD0,0xF3,0x21,0x19,0xD0,0x11,0x75,0xF9,0x01,0x67,0x01,
0xED,0xB0,0x21,0x4F,0xFA,0x22,0x66,0xF3,0x3E,0xC3,0x32,0x65,0xF3,0xFB,0xC9,0x32,
0xD0,0xFA,0xED,0x53,0xD2,0xFA,0xCD,0xB9,0xF9,0xD8,0xE5,0x21,0xD0,0xFA,0x06,0x04,
0xCD,0x22,0xFA,0x16,0x00,0xCD,0xD6,0xF9,0xE1,0xD8,0x3A,0xD0,0xFA,0xFE,0x52,0x28,
0x0C,0x06,0x00,0xCD,0x22,0xFA,0x06,0x00,0xCD,0x22,0xFA,0x18,0x03,0xCD,0xEA,0xF9,
0xEB,0x2A,0xD2,0xFA,0x23,0x22,0xD2,0xFA,0xEB,0x3A,0xCE,0xFA,0x3D,0x32,0xCE,0xFA,
0x20,0xC4,0xC9,0x16,0x0F,0xCD,0xD6,0xF9,0xD8,0x3E,0x0F,0xD3,0xA0,0xDB,0xA2,0xCB,
0xF7,0xE6,0xD3,0xD3,0xA1,0xCB,0xEF,0xD3,0xA1,0xCB,0xAF,0xF6,0x0C,0xD3,0xA1,0xC9,
0x01,0x00,0x10,0x3E,0x0E,0xD3,0xA0,0xDB,0xA2,0xE6,0x0F,0xBA,0xC8,0x0B,0x78,0xB1,
0x20,0xF5,0x37,0xC9,0xCD,0xED,0xF9,0x01,0x0F,0x00,0x16,0xDF,0x79,0xD3,0xA0,0xDB,
0xA2,0xCB,0xEF,0x5F,0x7B,0xD3,0xA1,0x3E,0x0E,0xD3,0xA0,0xDB,0xA2,0xED,0x67,0x79,
0xD3,0xA0,0x7B,0xA2,0xD3,0xA1,0x7B,0xD3,0xA1,0x3E,0x0E,0xD3,0xA0,0xDB,0xA2,0xED,
0x67,0x79,0xD3,0xA0,0x7B,0xA2,0xD3,0xA1,0x23,0x10,0xD9,0xC9,0x3E,0x0F,0xD3,0xA0,
0xDB,0xA2,0xE6,0xF3,0xCB,0xEF,0x5F,0x7E,0x07,0x07,0x57,0x48,0x06,0x04,0xE6,0x0C,
0xB3,0xD3,0xA1,0xE6,0xDF,0xD3,0xA1,0x7A,0x0F,0x0F,0x57,0x10,0xF1,0x41,0x23,0x10,
0xE6,0x7B,0xF6,0x0C,0xCB,0xAF,0xD3,0xA1,0xC9,0xF5,0xE5,0xD5,0xC5,0x21,0x08,0x00,
0x39,0x11,0xD4,0xFA,0x01,0x08,0x00,0x1A,0xED,0xA1,0x13,0xC2,0xAB,0xFA,0xEA,0x5D,
0xFA,0xF3,0xC1,0x78,0x32,0xCF,0xFA,0x32,0xCE,0xFA,0x3A,0x42,0xF3,0xCD,0xB2,0xFA,
0xD1,0xE1,0xF1,0x32,0xD1,0xFA,0x3E,0x57,0x38,0x02,0x3E,0x52,0xCD,0x75,0xF9,0xF5,
0x3A,0x48,0xF3,0xCD,0xB2,0xFA,0x3E,0x0F,0xD3,0xA0,0xDB,0xA2,0xCB,0xB7,0xD3,0xA1,
0xEB,0x21,0x14,0x00,0x39,0xF1,0xF9,0xEB,0x3E,0x02,0x06,0x00,0x38,0x05,0x3A,0xCF,
0xFA,0x47,0xB7,0xFB,0xC9,0xC1,0xD1,0xE1,0xF1,0xDB,0xA8,0xC9,0x47,0xE6,0x03,0x07,
0x07,0x4F,0xDB,0xA8,0xE6,0xF3,0xB1,0xD3,0xA8,0x78,0xE6,0x0C,0x4F,0x3A,0xFF,0xFF,
0x2F,0xE6,0xF3,0xB1,0x32,0xFF,0xFF,0xC9,0x00,0x00,0x00,0x00,0x00,0x00,0xEA,0x5F,
0xD0,0x5F,0xC5,0x5F,0xDC,0x76 };

void setup() {
  // use pin 13 LED as indicator
  pinMode(13,OUTPUT);
  // use port C for communication ((A0-A3)PC0-4->MSX; (A4-A5)PC4-5<-MSX) enable pull-up resistors
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  // clock input (pin 8(PB0) or pin2 (PD2, INT0))
  pinMode(8, INPUT_PULLUP); //polling, no interrupts yet
  // pin 9 for fsk output (tape); use timer1 for pwm output
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
  //pinMode(A6,INPUT);  // to be used as TAPE-IN (future)
  //analogReference(INTERNAL);  // use internal 1.1V ref
  pinMode(motorIntPin, INPUT_PULLUP);
  PORTC|= B00001111; // set pins HIGH

  // setup serial COM 1Mbps
  Serial.begin(1000000);
  sendmessage("Arduino active");
  // wait for MSX to connect (pin8 low)
  while ((PINB & 1) == 1) {}
  sendmessage("MSX connected");
  digitalWrite(13,LOW);
  delay(500); //wait for connection to settle
  attachInterrupt(digitalPinToInterrupt(motorIntPin), tapeout, FALLING);
}

void tapeout() {
  tape=1;
}

byte setupfsk() {
  // check variables
  timerinc=MARK;  // start with header
  if (intcnt==0) return false;  // header count cannot be zero
  if ((pos>endpos) || (endpos>16384)) return false; // end position must be >= startpos and fit in buffer
  // setup pwm
  bitcnt=255;
  TCCR1B=1; // use 16MHz clock
  TIMSK1=2; // enable int for compare
  TCCR1A=64;  // enable PWM
  TCNT1=0;  // reset timer
  OCR1A=timerinc; // set compare int value, start with 'LOW'
  return true;
}

SIGNAL(TIMER1_COMPA_vect) {

  intcnt--; // finish actual waveform
  if (!intcnt) {  // set next waveform   
    switch (bitcnt) {
      case 255:   // startbit 1x '0'
        timerinc=SPACE;
        intcnt=2;
        bitcnt++;
        break;
      case 8:   // stopbit 2x '1'
        timerinc=MARK;
        intcnt=8; // try to properly finish waveform
        pos++;
        bitcnt=(pos>endpos)?9:255;
        break;
      case 9:   // we are done, stop
        TCCR1A=0; // disable PWM
        return;
      default:  // data bits 0-7
        if (pgm_read_byte(&cas[pos])&(1<<bitcnt)) {
          timerinc=MARK;  // 1
          intcnt=4;
        }
        else {
          timerinc=SPACE; // 0
          intcnt=2;
        }
        bitcnt++;
    }
  }
  OCR1A+=timerinc;
  return;
}

// send data as fsk to pin9 in MSX format
void senddata() {
    digitalWrite(13,HIGH);  //indicate we are doing something
    //first part: long header, 10 x '0xD0' and 6 char filename
    delay(100);
    intcnt=32000;
    pos=0;
    endpos=HEADER-1;
    setupfsk();
    // wait until finished
    while (bitcnt!=9) {}
    //second part: short header, start, end ,exec and data
    delay(1150);
    intcnt=8000;
    pos=HEADER;
    endpos=sizeof(cas)-1;
    setupfsk();
    // wait until finished
    while (bitcnt!=9) {}
    delay(100);   // wait for motor relais to settle
    tape=0;
    digitalWrite(13,LOW);  //indicate we are done
    return;
}

void sendmessage(char *szText) {

  byte i,txt[4];
  txt[0]='C';
  for (i=1;i<4;i++) txt[i]=0;
  Serial.write(txt, sizeof(txt));
  Serial.flush();
  Serial.println(szText);
  Serial.flush();
  return;
}

int readsectorPC() {

  word bytes;
  long t0,t1;
  char timing[24];

  //digitalWrite(13,LOW); //start of read
  t0=millis();
  Serial.write(cmd, sizeof(cmd));
  Serial.flush();
  bytes = Serial.readBytes(sector, 512);
  t1=millis();
  sprintf(timing,"delay: %i ms.",t1-t0);
  sendmessage(timing);
  //digitalWrite(13,HIGH); //end of read
  if (bytes == 512) return 0;
  return 1;
}

int writesectorPC() {

  word bytes;

  Serial.write(cmd, sizeof(cmd));
  Serial.flush();
  bytes = Serial.write(sector, 512);
  Serial.flush();
  if (bytes == 512) return 0;
  return 1;
}

// checks command from MSX (0=valid ; 1=invalid command)
int checkcmd() {

  word nr;
  
  if (cmd[0]!='R' && cmd[0]!='W') return 1; // invalid command
  if (cmd[1]>1) return 1; // needs to be 0 (A:) or 1 (B:) 
  nr=cmd[3];
  nr=nr<<8;
  nr+=cmd[2];
  if (nr>=1440) return 1; // 720kB disk cannot have more sectors of 512B
  return 0;
}

void writesectorMSX() {

  word i;
  cli();
  //digitalWrite(13,LOW);
  PORTC&=B11110000; // pin1-4 of MSX LOW
  for (i=0;i<sizeof(sector);i++) {
      //wait for MSX clock to go LOW
      while ((PINB & 1) == 1) {}
      //wait for MSX clock to go HIGH; send on rising edge
      while ((PINB & 1) == 0) {}
      // send lower 4 bits
      PORTC=(PORTC & B11110000)|(sector[i]&B00001111);
      //wait for MSX clock to go LOW
      while ((PINB & 1) == 1) {}
      //wait for MSX clock to go HIGH; send on rising edge
      while ((PINB & 1) == 0) {}
      // send higher 4 bits
      PORTC=(PORTC & B11110000)|((sector[i]&B11110000)>>4);
  }
  //digitalWrite(13,HIGH);
  sei();
  return;
}

void readsectorMSX() {

  word i;
  byte j,msxbits,val;

  cli();
  PORTC&=B11110000; // pin1-4 of MSX LOW
  for (i=0;i<sizeof(sector);i++) {
    val=0;
    for (j=0;j<4;j++) {
      //wait for MSX clock to go HIGH
      while ((PINB & 1) == 0) {}
      //wait for MSX clock to go LOW (read on falling edge!)
      while ((PINB & 1) == 1) {}
      // read the 2 bits from MSX pin 6-7
      msxbits=(PINC&B00110000)>>4;
      // shift into sector byte (LSB first)
      val|=(msxbits<<(j+j));  
    }
    sector[i]=val;
  }
  sei();
  return;
}

// receive 4 command bytes from MSX
uint8_t receivecmdMSX() {

  byte i,j,msxbits,cmdbyte;
  
  //cli();
  for (i=0;i<4;i++) {
    cmdbyte=0;
    for (j=0;j<4;j++) {
      //wait for MSX clock to go HIGH (read on falling edge!)
      //while ((PINB & 1) == 0) {}
      if (WaitForClkHigh()) return 1;
      //wait for MSX clock to go LOW
      //while ((PINB & 1) == 1) {}
      if (WaitForClkLow()) return 1;
      // read the 2 bits from MSX pin 6-7
      msxbits=(PINC&B00110000)>>4;
      // shift into cmd byte (LSB first)
      cmdbyte|=(msxbits<<(j+j));  
    }
    cmd[i]=cmdbyte;
  }
  //sei();
  return 0;
}

uint8_t WaitForClkHigh() {
  TCNT0=0;
  while ((PINB & 1) == 0 && (TCNT0<50)) {}  //50 ~=200µs
  if (TCNT0>=50) return 1;  //time-out
  return 0;
}

uint8_t WaitForClkLow() {
  TCNT0=0;
  while ((PINB & 1) == 1 && (TCNT0<50)) {}  //50 ~=200µs
  if (TCNT0>=50) return 1;  //time-out
  return 0;
}

void loop() {
  // MSX command loop
  while (1) {
    while ((PINB & 1) == 1) {} // wait for MSX clock to go LOW
    digitalWrite(13,LOW); // no activity
    PORTC|=B00001111; // pin1-4 of MSX HIGH
    // wait for rising pulse on pin 8 or tape int
    while (((PINB & 1) == 0) && (tape==0)) {}
    if (tape) {
      senddata();
      continue;
    }
    // pin 6&7 of MSX low? -> valid start
    if ((PINC&B110000)!=0) continue;
    // wait for pin8 to go LOW
    if (WaitForClkLow()) continue;
    // receive command from MSX
    if (receivecmdMSX()) {
      sendmessage("wrong or no command");
      continue;
    }
    //sendmessage("command received");
    // valid command?
    if (checkcmd()) continue;
    //sendmessage("valid command received");
    digitalWrite(13,HIGH);  // yes, disk transfer
    // execute command
    if (cmd[0]=='R') {
      if (readsectorPC()) {
        sendmessage("read error from PC");
        continue;
      }
      writesectorMSX();
    }
    if (cmd[0]=='W') {
      readsectorMSX();
      writesectorPC();
    }
  }
}
