
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

#define s595_latch()    s595_STCP_LO; s595_STCP_HI
#define s595_reset()    s595_MR_LO; s595_MR_HI
#define s595_shift_LO() (PORTD =                  0); s595_SHCP_HI  //  s595_SHCP_LO; s595_DS_LO; s595_SHCP_HI //    
#define s595_shift_HI() (PORTD = (1 << s595_DS_bit)); s595_SHCP_HI  //  s595_SHCP_LO; s595_DS_HI; s595_SHCP_HI //   


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
  s595_reset();
  s595_on();

  configure_interrupts();
}

void configure_interrupts(void) {
  cli(); // disable interrupts

  /***************** I DO NOT KNOW WHAT I'M DOING HERE...! *******************/
  
  // Timer 1 (16 bit)
  TCCR1A =  (1 << WGM01);                // CTC mode
  TCCR1B =  (1 << CS10);// | (1 << CS10);   // prescaler; CS11 only yields about 250 µs (4 kHz)
  TIMSK1 |= (1 << OCIE1A);              // compare interrupt
  
  sei(); // enable interrupts
}

volatile long last_micros = 0;
volatile long delta_micros;

ISR(TIMER1_COMPA_vect) {
  long micros_now = micros();
  delta_micros = micros_now - last_micros;
  last_micros = micros_now;
}

volatile int i = 0;
int delay_ms = 100;

const byte data[] = {
  B11000000,
  B11100000,
  B01110000,
  B00111000,
  B00011100,
  B00001110,
  B00000111,
  B00000011,  B00000001,
  B00000011,
  B00000111,
  B00001110,
  B00011100,
  B00111000,
  B01110000,
  B11100000,
  B11000000
};

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
// if there's any serial available, read it:
  if (Serial) {
    Serial.println(delta_micros);
    while (Serial.available() > 0) {
      delay_ms = Serial.parseInt();
  
      if (Serial.read() == '\n') {
        Serial.print("delay: ");
        Serial.print(delay_ms);
        Serial.println("ms");
      }
    } 
  }
  
  shiftout_byte(data[i]);
  s595_latch();

  if (++i >= sizeof(data)) {
    i = 0;
  }
  delay(delay_ms);
  
}




