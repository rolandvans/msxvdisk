// read/write disk sectors (512 bytes)

// global vars (disk sector buffer and command buffer)
byte sector[512]; // disk sector buffer
byte cmd[4];  //bytes  0: 'R'/'W' 1: disk_nr 2: sector_nr low byte 3: sector_nr high byte

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
  // clock input (pin 8(PB0) or pin2 (PD2, INT0)
  pinMode(8, INPUT_PULLUP); //polling, no interrupts yet
  // pin 9 for fsk output (tape); use timer1 for pwm output
  pinMode(9,OUTPUT);
  PORTC|= B00001111; // set pins HIGH

  // setup serial COM 1Mbps
  Serial.begin(1000000);
  //Serial.begin(19200);
  // establish link with PC TODO

  /* fill buffer with dummy data
  for (i=0;i<512;i++) {
    sector[i]=170;
  }*/

  // wait for MSX to connect (pin8 low)
  //Serial.println("Arduino active");
  while ((PINB & 1) == 1) {}
  //Serial.println("MSX connected");
  digitalWrite(13,LOW);
  delay(500); //wait for connection to settle
}

int readsectorPC() {

  word bytes;
  
  Serial.write(cmd, sizeof(cmd));
  Serial.flush();
  bytes = Serial.readBytes(sector, 512);
  if (bytes == 512) return 0;
  return 1;
}

int writesectorPC() {

  word bytes;

  Serial.write(cmd, sizeof(cmd));
  Serial.flush();
  //delay(1); //optional delay
  bytes = Serial.write(sector, 512);
  Serial.flush();
  if (bytes == 512) return 0;
  return 1;
}

// checks command from MSX (0=valid ; 1=invalid command)
int checkcmd() {

  word nr;
  
  if (cmd[0]!='R' && cmd[0]!='W') return 1; // invalid command
  if (cmd[1]!=0) return 1; // needs to be zero
  //nr=cmd[2]+cmd[3]<<8; // sector number
  nr=cmd[3];
  nr=nr<<8;
  nr+=cmd[2];
  if (nr>=1440) return 1; // 720kB disk cannot have more sectors of 512B
  return 0;
}

void writesectorMSX() {

  word i;
  cli();
  PORTC&=B11110000; // pin1-4 of MSX LOW
  for (i=0;i<sizeof(sector);i++) {
      //wait for MSX clock to go LOW
      while ((PINB & 1) == 1) {}
      //wait for MSX clock to go HIGH
      while ((PINB & 1) == 0) {}
      // send lower 4 bits
      PORTC=(PORTC & B11110000)|(sector[i]&B00001111);
      //wait for MSX clock to go LOW
      while ((PINB & 1) == 1) {}
      //wait for MSX clock to go HIGH
      while ((PINB & 1) == 0) {}
      // send higher 4 bits
      PORTC=(PORTC & B11110000)|((sector[i]&B11110000)>>4);
  }
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
      //wait for MSX clock to go LOW
      while ((PINB & 1) == 0) {}
      //wait for MSX clock to go HIGH
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
void receivecmdMSX() {

  byte i,j,msxbits,cmdbyte;
  
  cli();
  for (i=0;i<4;i++) {
    cmdbyte=0;
    for (j=0;j<4;j++) {
      //wait for MSX clock to go LOW
      while ((PINB & 1) == 0) {}
      //wait for MSX clock to go HIGH
      while ((PINB & 1) == 1) {}
      // read the 2 bits from MSX pin 6-7
      msxbits=(PINC&B00110000)>>4;
      // shift into cmd byte (LSB first)
      cmdbyte|=(msxbits<<(j+j));  
    }
    cmd[i]=cmdbyte;
  }
  sei();
  //for (i=0;i<4;i++) Serial.println(cmd[i]);

  return;
}

void loop() {
  // MSX command loop
  while (1) {
    digitalWrite(13,LOW);
    PORTC|=B00001111; // pin1-4 of MSX HIGH
    // wait for rising pulse
    while ((PINB & 1) == 0) {}
    //digitalWrite(13,HIGH);
    // pin 6&7 of MSX low? -> valid start
    if ((PINC&B110000)!=0) {
      // no, wait for MSX clock to go LOW and restart
      while ((PINB & 1) == 1) {}
      continue;  
    }
    // wait for pin8 to go LOW
    while ((PINB & 1) == 1) {}
    // receive command from MSX
    receivecmdMSX();
    digitalWrite(13,HIGH);
    // valid command? TODO: check command
    // execute command
    if (cmd[0]=='R') {
      readsectorPC();
      // acknoledge to MSX
      //PORTC&=B11110000; // pin1-4 of MSX LOW
      writesectorMSX();
    }
    if (cmd[0]=='W') {
      // acknoledge to MSX
      //PORTC&=B11110000; // pin1-4 of MSX LOW
      readsectorMSX();
      writesectorPC();
    }
    // wait for MSX clock to go LOW
    while ((PINB & 1) == 1) {}
  }
}
