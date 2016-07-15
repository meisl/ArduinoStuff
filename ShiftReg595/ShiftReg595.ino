

#define s595_OE   12
#define s595_MR   11
#define s595_STCP 10
#define s595_SHCP  9
#define s595_DS    8

/*
#define s595_OE_LO   PORTB &= B11101111
#define s595_OE_HI   PORTB |= B00010000
#define s595_MR_LO   PORTB &= B11110111
#define s595_MR_HI   PORTB |= B00001000
#define s595_STCP_LO PORTB &= B11111011
#define s595_STCP_HI PORTB |= B00000100
#define s595_SHCP_LO PORTB &= B11111101
#define s595_SHCP_HI PORTB |= B00000010
*/

#define s595_OE_LO   digitalWrite(s595_OE,   LOW)
#define s595_OE_HI   digitalWrite(s595_OE,   HIGH)
#define s595_MR_LO   digitalWrite(s595_MR,   LOW)
#define s595_MR_HI   digitalWrite(s595_MR,   HIGH)
#define s595_STCP_LO digitalWrite(s595_STCP, LOW)
#define s595_STCP_HI digitalWrite(s595_STCP, HIGH)
#define s595_SHCP_LO digitalWrite(s595_SHCP, LOW)
#define s595_SHCP_HI digitalWrite(s595_SHCP, HIGH)
#define s595_DS_LO   digitalWrite(s595_DS,   LOW)
#define s595_DS_HI   digitalWrite(s595_DS,   HIGH)

#define s595_off()   s595_OE_HI
#define s595_on()    s595_OE_LO

#define s595_latch()    s595_STCP_LO; s595_STCP_HI
#define s595_reset()    s595_MR_LO; s595_MR_HI; s595_latch()
#define s595_shift_LO() s595_SHCP_LO; s595_DS_LO; s595_SHCP_HI   //  PORTB = B00001000; s595_SHCP_HI
#define s595_shift_HI() s595_SHCP_LO; s595_DS_HI; s595_SHCP_HI   //  PORTB = B00101001; s595_SHCP_HI




void setup() {
  
  pinMode(13,        OUTPUT);  // 
  pinMode(s595_OE,   OUTPUT);  // OE (active-low)
  pinMode(s595_MR,   OUTPUT);  // MR (active-low)
  pinMode(s595_STCP, OUTPUT);  // STCP ("latch-out")
  pinMode(s595_SHCP, OUTPUT);  // SHCP ("shift-out")
  pinMode(s595_DS,   OUTPUT);  // DS (serial data)
  //DDRB |= B00111111;  // pins 8 thru 13 as outputs

  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  digitalWrite( 9, LOW);
  digitalWrite( 8, LOW);
  
  s595_off();
  s595_reset();
  s595_on();
}

const int n = 8;
int i = n;

void loop() {
  if (i > 0) {
    s595_shift_HI();
    s595_latch();
    i--;
  } else {
    s595_reset();
    i = n;
  }
  delay(100);
}




