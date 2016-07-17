
//#include "CurieTimerOne.h"

#define s595_OE_pin    6  // PORTD7 on Leonardo
#define s595_OE_port   PORTD
#define s595_OE_bit    7

#define s595_MR_pin    5  // PORTC6 on Leonardo
#define s595_MR_port   PORTC
#define s595_MR_bit    6

#define s595_STCP_pin  4  // PORTD4 on Leonardo
#define s595_STCP_port PORTD
#define s595_STCP_bit  4

#define s595_SHCP_pin  3  // PORTD0 on Leonardo
#define s595_SHCP_port PORTD
#define s595_SHCP_bit  0

#define s595_DS_pin    2  // PORTD1 on Leonardo
#define s595_DS_port   PORTD
#define s595_DS_bit    1

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

#define s595_off()   s595_OE_HI
#define s595_on()    s595_OE_LO

#define s595_latch()    s595_STCP_LO; s595_STCP_HI; (s595_state_ext = s595_state_int)
#define s595_clear()    s595_MR_LO; s595_MR_HI; (s595_state_int = 0)
#define s595_shift_LO() (PORTD =                  0); s595_SHCP_HI; (s595_state_int <<= 1)  //  s595_SHCP_LO; s595_DS_LO; s595_SHCP_HI //    
#define s595_shift_HI() (PORTD = (1 << s595_DS_bit)); s595_SHCP_HI; (s595_state_int <<= 1); (s595_state_int |= 1);   //  s595_SHCP_LO; s595_DS_HI; s595_SHCP_HI //   


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
#define PRESCALE_BY_1     (1 << CS10)                 //  16 MHz     /   62.5 ns
#define PRESCALE_BY_8     (1 << CS11)                 //   2 MHz     /  500.0 ns
#define PRESCALE_BY_64    (1 << CS11) | (1 << CS10)   // 250 KHz     /    4.0 µs
#define PRESCALE_BY_256   (1 << CS12)                 //  62.5 KHz   /   16.0 µs
#define PRESCALE_BY_1024  (1 << CS12) | (1 << CS10)   //  15.625 KHz /   64.0 µs

void configure_interrupts(void) {
  cli(); // disable interrupts
  
  // Timer 1 (16 bit)
  TCCR1A =  ((1 << WGM11) & 0x00) | ((1 << WGM10) & 0x00);                    // CTC mode
  TCCR1B =  ((1 << WGM13) & 0x00) | ((1 << WGM12) & 0xFF) | PRESCALE_BY_64;   // CTC mode, prescaling
  TIMSK1 |= (1 << OCIE1A);      // enable compare interrupt
  OCR1A  = 0; // (15625 >> 1);  // compare match register
    
  sei(); // enable interrupts
}
//1048576 micros, pwm_tick: 16, OCR1A: 62500, OCR1B: 0


const byte data[][8] = {
  { 0, 1, 5, 7, 9, 11, 99, 255 }
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

volatile long last_micros = 0;
volatile long delta_micros;

volatile boolean timer_intr    = false;
volatile boolean timer_enabled = false;

const    byte pwm_tickCount = 64;
volatile byte pwm_tick = pwm_tickCount;
const    int  pwm_rowCount = 8;
volatile int  pwm_row_index = pwm_rowCount;
const    int  pwm_columnCount = 8;

ISR(TIMER1_COMPA_vect) {
  if (!timer_enabled) {
    return;
  }
  long micros_now = micros();
  //delta_micros = micros_now - last_micros;
  //last_micros = micros_now;


  s595_latch(); // latch out what we've done last time
  s595_clear();


  if (++pwm_tick >= pwm_tickCount) {
    pwm_tick = 0;
    if (++pwm_row_index >= pwm_rowCount) {
      pwm_row_index = 0;
    }
    //rowData = data[pwm_row_index];
  }

  if (pwm_row_index == 0) {
    for (int i = 0; i < pwm_columnCount; i++) {
      byte x = data[0][i];
      if (x <= pwm_tick) {
        s595_shift_LO();
      } else {
        s595_shift_HI();
      }
    }
    delta_micros = micros() - micros_now;
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
      timer_intr = false;
      Serial.print(delta_micros);
      Serial.print(" micros");
      Serial.print(", s595_int: ");
      Serial.print(ltoa((1 << 8) | s595_state_int, buf, 2));
      Serial.print(", s595_ext: ");
      Serial.print(ltoa((1 << 8) | s595_state_ext, buf, 2));
      Serial.print(", pwm_tick: ");
      Serial.print(pwm_tick);
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




