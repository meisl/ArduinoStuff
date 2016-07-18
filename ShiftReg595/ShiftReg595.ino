#include <Arduino.h>

//#include "CurieTimerOne.h"

#define s595_OE_pin    6  // pin 6 is PORTD7 on Leonardo
#define s595_OE_port   PORTD
#define s595_OE_bit    7

#define s595_MR_pin    5  // pin 5 is PORTC6 on Leonardo
#define s595_MR_port   PORTC
#define s595_MR_bit    6

#define s595_STCP_pin  4  // pin 4 is PORTD4 on Leonardo
#define s595_STCP_port PORTD
#define s595_STCP_bit  4

#define s595_SHCP_pin  3  // pin 3 is PORTD0 on Leonardo
#define s595_SHCP_port PORTD
#define s595_SHCP_bit  0

#define s595_DS_pin    2  // pin 2 is PORTD1 on Leonardo
#define s595_DS_port   PORTD
#define s595_DS_bit    1

#define s595_DSb_pin    0  // pin 0 is PORTD2 on Leonardo
#define s595_DSb_port   PORTD
#define s595_DSb_bit    2

#define s595_DSc_pin    1  // pin 1 is PORTD3 on Leonardo
#define s595_DSc_port   PORTD
#define s595_DSc_bit    3



/*
D7 ~> PORTE6 on Leonardo
D0 ~> PORTD2 on Leonardo
D1 ~> PORTD3 on Leonardo
*/

volatile long s595_state_int = 0;
volatile long s595_state_ext = 0;

#define s595_OE_LO   (s595_OE_port   &= ~(1 << s595_OE_bit  ))  // digitalWrite(s595_pin_OE,   LOW)
#define s595_OE_HI   (s595_OE_port   |=  (1 << s595_OE_bit  ))  // digitalWrite(s595_OE_pin,   HIGH)
#define s595_MR_LO   (s595_MR_port   &= ~(1 << s595_MR_bit  ))  // digitalWrite(s595_MR_pin,   LOW)
#define s595_MR_HI   (s595_MR_port   |=  (1 << s595_MR_bit  ))  // digitalWrite(s595_MR_pin,   HIGH)
#define s595_STCP_LO (s595_STCP_port &= ~(1 << s595_STCP_bit))  // digitalWrite(s595_STCP_pin, LOW)
#define s595_STCP_HI (s595_STCP_port |=  (1 << s595_STCP_bit))  // digitalWrite(s595_STCP_pin, HIGH)
#define s595_SHCP_LO (s595_SHCP_port &= ~(1 << s595_SHCP_bit))  // digitalWrite(s595_SHCP_pin, LOW)
#define s595_SHCP_HI (s595_SHCP_port |=  (1 << s595_SHCP_bit))  // digitalWrite(s595_SHCP_pin, HIGH)
#define s595_DS_LO   (s595_DS_port   &= ~(1 << s595_DS_bit  ))  // digitalWrite(s595_DS_pin,   LOW)
#define s595_DS_HI   (s595_DS_port   |=  (1 << s595_DS_bit  ))  // digitalWrite(s595_DS_pin,   HIGH)
#define s595_DSa_LO  (s595_DS_port  &= ~(1 << s595_DS_bit  ))  // digitalWrite(s595_DS_pin,   LOW)
#define s595_DSa_HI  (s595_DS_port  |=  (1 << s595_DS_bit  ))  // digitalWrite(s595_DS_pin,   HIGH)
#define s595_DSb_LO  (s595_DSb_port  &= ~(1 << s595_DSb_bit  ))
#define s595_DSb_HI  (s595_DSb_port  |=  (1 << s595_DSb_bit  ))
#define s595_DSc_LO  (s595_DSc_port  &= ~(1 << s595_DSc_bit  ))
#define s595_DSc_HI  (s595_DSc_port  |=  (1 << s595_DSc_bit  ))

#define s595_off()   s595_OE_HI
#define s595_on()    s595_OE_LO

#define s595_latch()    s595_STCP_LO; s595_STCP_HI; //(s595_state_ext = s595_state_int)
#define s595_clear()    s595_MR_LO; s595_MR_HI; //(s595_state_int = 0)
#define s595_shift_LO() (PORTD =                  0); s595_SHCP_HI; //(s595_state_int <<= 1)  //  s595_SHCP_LO; s595_DS_LO; s595_SHCP_HI //    
#define s595_shift_HI() (PORTD = (1 << s595_DS_bit)); s595_SHCP_HI; //(s595_state_int <<= 1); (s595_state_int |= 1);   //  s595_SHCP_LO; s595_DS_HI; s595_SHCP_HI //   


void setup() {
  Serial.begin(57600);

  DDRD |= B01011111;
  
  pinMode(s595_OE_pin,   OUTPUT);  // OE (active-low)
  pinMode(s595_MR_pin,   OUTPUT);  // MR (active-low)
  pinMode(s595_STCP_pin, OUTPUT);  // STCP ("latch-out")
  pinMode(s595_SHCP_pin, OUTPUT);  // SHCP ("shift-out")
  pinMode(s595_DS_pin,   OUTPUT);  // DS (serial data)

  digitalWrite(s595_OE_pin, HIGH);
  digitalWrite(s595_MR_pin, LOW);
  digitalWrite(s595_STCP_pin, LOW);
  digitalWrite(s595_SHCP_pin, LOW);
  digitalWrite(s595_DS_pin, LOW);
  
  s595_off();
  s595_clear();
  s595_latch();
  s595_on();

  configure_interrupts();
}

#define PRESCALE_STOPPED  0
#define PRESCALE_BY_1     (1 << CS10)                 //  16.000 MHz /  62.5 ns
#define PRESCALE_BY_8     (1 << CS11)                 //   2.000 MHz / 500.0 ns
#define PRESCALE_BY_64    (1 << CS11) | (1 << CS10)   // 250.000 KHz /   4.0 µs
#define PRESCALE_BY_256   (1 << CS12)                 //  62.500 KHz /  16.0 µs
#define PRESCALE_BY_1024  (1 << CS12) | (1 << CS10)   //  15.625 KHz /  64.0 µs

void configure_interrupts(void) {
  noInterrupts();
  
  // Timer 1 (16 bit)
  TCCR1A =  ((1 << WGM11) & 0x00) | ((1 << WGM10) & 0x00);                    // CTC mode
  TCCR1B =  ((1 << WGM13) & 0x00) | ((1 << WGM12) & 0xFF) | PRESCALE_BY_64;   // CTC mode, prescaling
  TIMSK1 |= (1 << OCIE1A);      // enable compare interrupt
  OCR1A  = 12; // (15625 >> 1);  // compare match register
    
  interrupts();
}


#define  pwm_tickCount   32
#define  pwm_columnCount 24
#define  pwm_rowCount    10

byte data[][32] = {
  { 0, 255, 0, 255, 0, 255, 0, 255,     0, 255, 0, 255, 0, 255, 0, 255,     0, 255, 0, 255, 0, 255, 0, 255,     0, 1, 2, 9, 13, 5, 21, 65 }
  //{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }  
};

byte data_rgb[][pwm_columnCount*3] = {
  { 
    0x00, 0x00, 0x00,   0xFF, 0xFF, 0xFF,  0x00, 0x00, 0x00,   0xFF, 0xFF, 0xFF,  0x00, 0x00, 0x00,   0xFF, 0xFF, 0xFF,  0x00, 0x00, 0x00,   0xFF, 0xFF, 0xFF,       
    0x00, 0x00, 0x00,   0xFF, 0xFF, 0xFF,  0x00, 0x00, 0x00,   0xFF, 0xFF, 0xFF,  0x00, 0x00, 0x00,   0xFF, 0xFF, 0xFF,  0x00, 0x00, 0x00,   0xFF, 0xFF, 0xFF,       
    0x00, 0x00, 0x00,   0x01, 0xFF, 0xFF,  0x02, 0x00, 0x00,   0x09, 0xFF, 0xFF,  0x0D, 0x00, 0x00,   0xF5, 0xFF, 0xFF,  0x15, 0x00, 0x00,   0x1F, 0xFF, 0xFF,       
  }
  //{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }  
};

const byte data2[] = {
  B11000000,  //  { 0x08, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, //  
  B11100000,  //  { 0x08, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, //  
  B01110000,
  B00111000,
  B00011100,
  B00001110,
  B00000111,
  B00000011,  
  B00000001,
  B00000011,
  B00000111,
  B00001110,
  B00011100,
  B00111000,
  B01110000,
  B11100000,
  B11000000
};

unsigned int  delta_micros;
unsigned int  delta_micros2;

boolean timer_intr    = false;
boolean timer_enabled = false;


byte pwm_tick = pwm_tickCount;
int  pwm_row_index = pwm_rowCount;


byte rowBuf[(pwm_tickCount * pwm_columnCount) >> 3];



ISR(TIMER1_COMPA_vect) {
  if (!timer_enabled) {
    return;
  }
  long micros_now = micros();


  s595_latch(); // latch out what we've done last time
  s595_clear();


  if (++pwm_tick >= pwm_tickCount) {
    if (++pwm_row_index >= pwm_rowCount) {
      pwm_row_index = 0;
    }
    /*
    rowData = &rowBuf[pwm_row_index];
    for (pwm_tick = 0; pwm_tick < pwm_tickCount; pwm_tick++) {
      byte* p = &data[pwm_row_index][0];

      *rowData++ = (*p++ > pwm_tick ? 0x01 : 0) | (*p++ > pwm_tick ? 0x02 : 0) | (*p++ > pwm_tick ? 0x04 : 0) | (*p++ > pwm_tick ? 0x08 : 0)
                 | (*p++ > pwm_tick ? 0x10 : 0) | (*p++ > pwm_tick ? 0x20 : 0) | (*p++ > pwm_tick ? 0x40 : 0) | (*p++ > pwm_tick ? 0x80 : 0);

      *rowData++ = (*p++ > pwm_tick ? 0x01 : 0) | (*p++ > pwm_tick ? 0x02 : 0) | (*p++ > pwm_tick ? 0x04 : 0) | (*p++ > pwm_tick ? 0x08 : 0)
                 | (*p++ > pwm_tick ? 0x10 : 0) | (*p++ > pwm_tick ? 0x20 : 0) | (*p++ > pwm_tick ? 0x40 : 0) | (*p++ > pwm_tick ? 0x80 : 0);

      *rowData++ = (*p++ > pwm_tick ? 0x01 : 0) | (*p++ > pwm_tick ? 0x02 : 0) | (*p++ > pwm_tick ? 0x04 : 0) | (*p++ > pwm_tick ? 0x08 : 0)
                 | (*p++ > pwm_tick ? 0x10 : 0) | (*p++ > pwm_tick ? 0x20 : 0) | (*p++ > pwm_tick ? 0x40 : 0) | (*p++ > pwm_tick ? 0x80 : 0);

      *rowData++ = (*p++ > pwm_tick ? 0x01 : 0) | (*p++ > pwm_tick ? 0x02 : 0) | (*p++ > pwm_tick ? 0x04 : 0) | (*p++ > pwm_tick ? 0x08 : 0)
                 | (*p++ > pwm_tick ? 0x10 : 0) | (*p++ > pwm_tick ? 0x20 : 0) | (*p++ > pwm_tick ? 0x40 : 0) | (*p++ > pwm_tick ? 0x80 : 0);

      //*rowData++ = (*p++ > pwm_tick ? 0x80 : 0) | (*p++ > pwm_tick ? 0x40 : 0) | (*p++ > pwm_tick ? 0x20 : 0) | (*p++ > pwm_tick ? 0x10 : 0)
      //           | (*p++ > pwm_tick ? 0x08 : 0) | (*p++ > pwm_tick ? 0x04 : 0) | (*p++ > pwm_tick ? 0x02 : 0) | (*p++ > pwm_tick ? 0x01 : 0);

    }
    */
    pwm_tick = 0;
    //rowData = &rowBuf[pwm_row_index];
  }

  //char buf[33];

  byte* rowData = (byte*)&data_rgb[pwm_row_index];
  if (pwm_row_index == 0) {

    for (int i = 0; i < pwm_columnCount; i++) {
      /*
      byte x = data[0][i];  //  
      if (x <= pwm_tick) {
        s595_shift_LO();
      } else {
        s595_shift_HI();
      }
      */
      /*
      if (*rowData++ <= pwm_tick) {
        s595_shift_LO();
      } else {
        s595_shift_HI();
      } 
      */
      byte out = 0;
      if (*rowData++ > pwm_tick) {
        out |= (1 << s595_DS_bit);  // "bank" a
      }
      if (*rowData++ > pwm_tick) {  // "bank" b
        out |= (1 << s595_DSb_bit);
      }
      if (*rowData++ > pwm_tick) {  // "bank" c
        out |= (1 << s595_DSc_bit);
      }
      PORTD = out; // also sets SHCP and STCP low
      s595_SHCP_HI;
    }
/*
    byte x;

    x = *rowData++;
    if (x & 0x80) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x40) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x20) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x10) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x08) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x04) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x02) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x01) { s595_shift_HI(); } else { s595_shift_LO(); }
    
    x = *rowData++;
    if (x & 0x80) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x40) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x20) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x10) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x08) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x04) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x02) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x01) { s595_shift_HI(); } else { s595_shift_LO(); }
    
    x = *rowData++;
    if (x & 0x80) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x40) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x20) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x10) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x08) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x04) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x02) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x01) { s595_shift_HI(); } else { s595_shift_LO(); }
    
    x = *rowData++;
    if (x & 0x80) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x40) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x20) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x10) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x08) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x04) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x02) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x01) { s595_shift_HI(); } else { s595_shift_LO(); }
*/
/*  
    x = *rowData++;
    if (x & 0x01) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x02) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x04) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x08) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x10) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x20) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x40) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x80) { s595_shift_HI(); } else { s595_shift_LO(); }
    
    x = *rowData++;
    if (x & 0x01) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x02) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x04) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x08) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x10) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x20) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x40) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x80) { s595_shift_HI(); } else { s595_shift_LO(); }
    
    x = *rowData++;
    if (x & 0x01) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x02) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x04) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x08) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x10) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x20) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x40) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x80) { s595_shift_HI(); } else { s595_shift_LO(); }
    
    x = *rowData++;
    if (x & 0x01) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x02) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x04) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x08) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x10) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x20) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x40) { s595_shift_HI(); } else { s595_shift_LO(); }
    if (x & 0x80) { s595_shift_HI(); } else { s595_shift_LO(); }
*/  
    unsigned long delta = micros() - micros_now;
    delta_micros = (unsigned int)delta;

/*
    Serial.print("tick ");
    Serial.print(pwm_tick); Serial.print(": ");
    Serial.print(delta_micros); Serial.print(" micros; 0x");
    Serial.print(ltoa( *((unsigned long*)(rowData - 4)), buf, 16));
    Serial.println();
*/
  }

  timer_intr = true;
}



volatile int i = 0;
int delay_ms = 1;

void shiftout_byte(byte b) {
  byte mask = 0x80;
  for (mask = 0x80; mask != 0; mask >>= 1) {
    if (b & mask) {
      s595_shift_HI();
    } else {
      s595_shift_LO();
    }
  }
}

  
void loop() {
  char buf[33];

// if there's any serial available, read it:
  if (Serial) {
    timer_enabled = true;
    if (timer_intr) {
      unsigned long micros = delta_micros;
               byte tick   = pwm_tick;
      timer_intr = false;
      Serial.print(micros);
      Serial.print(" micros");
      /*
      Serial.print(", s595_int: ");
      Serial.print(ltoa(((unsigned long)1 << pwm_columnCount) | s595_state_int, buf, 2));
      Serial.print(", s595_ext: ");
      Serial.print(ltoa(((unsigned long)1 << pwm_columnCount) | s595_state_ext, buf, 2));
      */
      Serial.print(", pwm_tick: ");
      Serial.print(tick);
      Serial.println();
    }
    while (Serial.available() > 0) {
      delay_ms = Serial.parseInt();
  
      if (Serial.read() == '\n') {
        Serial.print("delay: ");
        Serial.print(delay_ms);
        Serial.println("ms");
      }
    } 
  }

  if (delay_ms == 0) {
    s595_off();
    i = 0;
  } else {
    /*
    shiftout_byte(data[i]);
    s595_latch();
    s595_on();
    i++;

    if (i >= sizeof(data)) {
      i = 0;
    }
    delay(delay_ms);
    */
  }

  
}




