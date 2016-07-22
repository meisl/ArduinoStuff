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

#define TIMER1_PRESCALE_STOPPED 0
#define TIMER1_PRESCALE_BY_1    (                       _BV(CS10))  //  16.000 MHz /  62.5 ns
#define TIMER1_PRESCALE_BY_8    (           _BV(CS11)            )  //   2.000 MHz / 500.0 ns
#define TIMER1_PRESCALE_BY_64   (           _BV(CS11) | _BV(CS10))  // 250.000 KHz /   4.0 µs
#define TIMER1_PRESCALE_BY_256  (_BV(CS12)                          //  62.500 KHz /  16.0 µs
#define TIMER1_PRESCALE_BY_1024 (_BV(CS12)            | _BV(CS10))  //  15.625 KHz /  64.0 µs

#define TIMER3_PRESCALE_STOPPED 0
#define TIMER3_PRESCALE_BY_1    (                       _BV(CS30))  //  16.000 MHz /  62.5 ns
#define TIMER3_PRESCALE_BY_8    (           _BV(CS31)            )  //   2.000 MHz / 500.0 ns
#define TIMER3_PRESCALE_BY_64   (           _BV(CS31) | _BV(CS30))  // 250.000 KHz /   4.0 µs
#define TIMER3_PRESCALE_BY_256  (_BV(CS32)                          //  62.500 KHz /  16.0 µs
#define TIMER3_PRESCALE_BY_1024 (_BV(CS32)            | _BV(CS30))  //  15.625 KHz /  64.0 µs

#define TIMER2_PRESCALE_STOPPED 0
#define TIMER2_PRESCALE_BY_1    (                            (1 << CS20)) //  16.000 MHz /  62.5 ns
#define TIMER2_PRESCALE_BY_8    (              (1 << CS21)              ) //   2.000 MHz / 500.0 ns
#define TIMER2_PRESCALE_BY_32   (              (1 << CS21) | (1 << CS20)) // 500.000 KHz /   2.0 µs
#define TIMER2_PRESCALE_BY_64   ((1 << CS22)                            ) // 250.000 MHz /   4.0 µs 
#define TIMER2_PRESCALE_BY_128  ((1 << CS22)               | (1 << CS20)) // 125.000 MHz /   8.0 µs
#define TIMER2_PRESCALE_BY_256  ((1 << CS22) | (1 << CS21)              ) // 62.500 KHz /   16.0 µs
#define TIMER2_PRESCALE_BY_1024 ((1 << CS22) | (1 << CS21) | (1 << CS20)) // 15.625 KHz /   64.0 µs

void configure_interrupts(void) {
  noInterrupts();
  
  // Timer 1 (16 bit)
  TCCR1A =  ((1 << WGM11) & 0x00) | ((1 << WGM10) & 0x00);                          // TIMER1 CTC mode ("Clear Timer on Compare")
  TCCR1B =  ((1 << WGM13) & 0x00) | ((1 << WGM12) & 0xFF) | TIMER1_PRESCALE_BY_1;   // TIMER1 CTC mode, prescaling
  TIMSK1 |= (1 << OCIE1A);  // enable compare interrupt for TIMER1
  OCR1A  = (50 * 16) - 1;   // compare match register

  // Timer 3 (16 bit)
  TCCR3A =  ((1 << WGM31) & 0x00) | ((1 << WGM30) & 0x00);                        // TIMER3 normal mode
  TCCR3B =  ((1 << WGM33) & 0x00) | ((1 << WGM32) & 0x00) | TIMER3_PRESCALE_BY_1; // TIMER3 normal mode @ 16MHz / 62.5ns period (overflow after 4096 µs)
  TIMSK3 = 0; // no interrupt from Timer3

/* there is no TIMER2 on the Atmega32uXX
  TCCR2 = 0 | TIMER2_PRESCALE_BY_8; // TIMER2 in normal mode @ 2MHz / 0.5µs period (overflow after 128 µs)
  TIMSK2 &= ~(_BV(TOIE2) | _BV(OCIE2A) | _BV(OCIE2B)); // no interrupts from TIMER2
*/ 
  interrupts();
}


#define  pwm_tickCount   32
#define  pwm_columnCount 24
#define  pwm_rowCount    12

byte data[288];

byte data_rgb[][24*3] = {
  { 
    0x00, 0x00, 0x00,   0x01, 0x01, 0x01,   0x02, 0x02, 0x02,   0x09, 0x09, 0x09,   0x0D, 0x0D, 0x0D,   0x18, 0x18, 0x18,   0x1F, 0x1F, 0x1F,   0x02, 0x02, 0x02,
    0x00, 0x00, 0x00,   0x01, 0x01, 0x01,   0x02, 0x02, 0x02,   0x09, 0x09, 0x09,   0x0D, 0x0D, 0x0D,   0x18, 0x18, 0x18,   0x1F, 0x1F, 0x1F,   0x02, 0x02, 0x02,
    0x00, 0x00, 0x00,   0x01, 0x01, 0x01,   0x02, 0x02, 0x02,   0x09, 0x09, 0x09,   0x0D, 0x0D, 0x0D,   0x18, 0x18, 0x18,   0x1F, 0x1F, 0x1F,   0x02, 0x02, 0x02
  }
};

byte data_rgbX[][16*3] = {
  { 
    0x00, 0x00,   0x01, 0xFF,   0x02, 0x00,   0x09, 0xFF,   0x0D, 0x00,   0x18, 0xFF,   0x1F, 0x00,   0x02, 0xFF,       
    0x00, 0x00,   0x01, 0xFF,   0x02, 0x00,   0x09, 0xFF,   0x0D, 0x00,   0x18, 0xFF,   0x1F, 0x00,   0x02, 0xFF,       
    0x00, 0x00,   0x01, 0xFF,   0x02, 0x00,   0x09, 0xFF,   0x0D, 0x00,   0x18, 0xFF,   0x1F, 0x00,   0x02, 0xFF
  }
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

volatile uint16_t _tm_0;
volatile uint16_t _tm_1;
volatile uint16_t _missed;
volatile uint8_t  _pwm_tick;

uint16_t  missed;

bool timer_intr    = false;
bool timer_enabled = false;

byte pwm_tick = pwm_tickCount;
byte pwm_row_index = pwm_rowCount;

byte rowBuf[(pwm_tickCount * pwm_columnCount) >> 3];

ISR(TIMER1_COMPA_vect) {
  TCNT3 = 0;
  if (!timer_enabled) {
    return;
  }
 
  s595_latch(); // latch out what we've done last time
  s595_clear();

  if (++pwm_tick >= pwm_tickCount) {
    if (++pwm_row_index >= pwm_rowCount) {
      pwm_row_index = 0;
    }
    pwm_tick = 0;
  }


  if (pwm_row_index == 0) {


    
#define pwm_tick_ASM1

#ifdef pwm_tick_ASM1
  volatile byte* rowData = (byte*)&data_rgb[pwm_row_index];
  byte tick = pwm_tick;
  byte col = pwm_columnCount;
  TCNT3 = 0;
  asm volatile("                            ; cycles  // comment                        \n"
        "         sts 0x0095, r1            ; 2   // TCNT3H \n"
        "         sts 0x0094, r1            ; 2   // TCNT3L \n"
        "pwm_tick_loop:                     ;                                           \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch blue channel value       \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < blue channel value?     \n" 
        "         brlo rgb_XX1              ; 1/2     // if so use a 1 for blue...      \n"
        "rgb_XX0:                           ;         // ...otherwise a 0               \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch green channel value      \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < green channel value?    \n" 
        "         brlo rgb_X10              ; 1/2     // if so use a 1 for green...     \n"
        "rgb_X00:                           ;         // ...otherwise a 0               \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch red channel value        \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < red channel value?      \n" 
        "         brlo rgb_100              ; 1/2     // if so use a 1 for red...       \n"
        "rgb_000:                           ;         // ...otherwise a 0               \n"
        "         out  %[port], __zero_reg__; 2       // %[port] <- rgb, SHCP lo, STCP lo    \n"
        "         rjmp pwm_clock_out        ; 2                                         \n"
        "rgb_100:                           ;                                           \n"
        "         ldi  %[rgb], %[rgb_100]   ; 1       // %[port] <- 0b100, SHCP lo, STCP lo  \n"
        "         rjmp pwm_data_out         ; 2                                         \n"
        "rgb_XX1:                           ;                                           \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch green channel value      \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < green channel value?    \n" 
        "         brlo rgb_X11              ; 1/2     // if so use a 1 for green...     \n"
        "rgb_X01:                           ;         // ...otherwise a 0               \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch red channel value        \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < red channel value?      \n" 
        "         brlo rgb_101              ; 1/2     // if so use a 1 for red...       \n"
        "rgb_001:                           ;         // ...otherwise a 0               \n"
        "         ldi  %[rgb], %[rgb_001]   ; 1       // %[port] <- rgb, SHCP lo, STCP lo    \n"
        "         rjmp pwm_data_out         ; 2                                         \n"
        "rgb_101:                           ;                                           \n"
        "         ldi  %[rgb], %[rgb_101]   ; 1       // %[port] <- rgb, SHCP lo, STCP lo    \n"
        "         rjmp pwm_data_out         ; 2                                         \n"
        "rgb_X10:                           ;         // ...otherwise a 0               \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch red channel value        \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < red channel value?      \n" 
        "         brlo rgb_110              ; 1/2     // if so use a 1 for red...       \n"
        "rgb_010:                           ;         // ...otherwise a 0               \n"
        "         ldi  %[rgb], %[rgb_010]   ; 1       // %[port] <- rgb, SHCP lo, STCP lo    \n"
        "         rjmp pwm_data_out         ; 2                                         \n"
        "rgb_110:                           ;                                           \n"
        "         ldi  %[rgb], %[rgb_110]   ; 1       // %[port] <- rgb, SHCP lo, STCP lo    \n"
        "         rjmp pwm_data_out         ; 2                                         \n"
        "rgb_X11:                           ;         // ...otherwise a 0               \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch red channel value        \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < red channel value?      \n" 
        "         brlo rgb_111              ; 1/2     // if so use a 1 for red...       \n"
        "rgb_011:                           ;         // ...otherwise a 0               \n"
        "         ldi  %[rgb], %[rgb_011]   ; 1       // %[port] <- rgb, SHCP lo, STCP lo    \n"
        "         rjmp pwm_data_out         ; 2                                         \n"
        "rgb_111:                           ;                                           \n"
        "         ldi  %[rgb], %[rgb_111]   ; 1       // %[port] <- rgb, SHCP lo, STCP lo    \n"
        "pwm_data_out:                      ;                                           \n"
        "         out  %[port], %[rgb]      ; 1       // write rgb data to port (with SHCP = STCP = LO) \n"
        "pwm_clock_out:                     ;                                           \n"
        "         sbi  %[port], %[shcp_bit] ; 2       // SHCP hi on PORTD               \n"
        "         dec  %[col]               ; 1       // next column                    \n" 
        "         brne pwm_tick_loop        ; 1/2     // repeat if %[col] still > 0     \n"
        : [col]      "+r"  (col), // "r" means any register
                     "+e"  (rowData) // %a1
        : [tick]     "r"   (tick), 
          [port]     "I"   (_SFR_IO_ADDR(s595_DS_port)),
          [rgb]      "a"   (0),  // "a" means simple upper register (r16..r23)
          [rgb_001]  "I"   (                                         _BV(s595_DS_bit) ),
          [rgb_010]  "I"   (                     _BV(s595_DSb_bit)                    ),
          [rgb_011]  "I"   (                     _BV(s595_DSb_bit) | _BV(s595_DS_bit) ),
          [rgb_100]  "I"   ( _BV(s595_DSc_bit)                                        ),
          [rgb_101]  "I"   ( _BV(s595_DSc_bit)                     | _BV(s595_DS_bit) ),
          [rgb_110]  "I"   ( _BV(s595_DSc_bit) | _BV(s595_DSb_bit)                    ),
          [rgb_111]  "I"   ( _BV(s595_DSc_bit) | _BV(s595_DSb_bit) | _BV(s595_DS_bit) ),
          [shcp_bit] "I"   (s595_SHCP_bit)
    );
#endif
// ----------------------------------------

#ifdef pwm_tick_ASM2
    volatile byte* rowData = (byte*)&data_rgb[pwm_row_index];
    byte tick = pwm_tick;
    byte col = pwm_columnCount;
    TCNT3 = 0;
    asm volatile("                          ; cycles  // comment                        \n"
        "         sts 0x0095, r1            ; 2   // TCNT3H \n"
        "         sts 0x0094, r1            ; 2   // TCNT3L \n"
        "pwm_tick_loop:                                                                 \n"
        "         ldi  %[rgb], %[rgb_111]   ; 1       // start with all set to 1        \n"
        "blue:                                                                          \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch blue channel value       \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < blue channel value?     \n" 
        "         brlo green                ; 1/2     // if so use leave the 1 for blue \n"
        "         andi %[rgb], %[rgb_110]   ; 1       // ...otherwise clear blue bit    \n"
        "green:                                                                         \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch green channel value      \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < green channel value?    \n" 
        "         brlo red                ; 1/2       // if so use leave the 1 for green \n"
        "         andi %[rgb], %[rgb_101]   ; 1       // ...otherwise clear blue bit    \n"
        "red:                                                                           \n"
        "         ld   __tmp_reg__, %a1+    ; 2       // fetch red channel value        \n" 
        "         cp   %[tick], __tmp_reg__ ; 1       // tick < red channel value?      \n" 
        "         brlo pwm_data_out         ; 1/2     // if so use leave the 1 for red  \n"
        "         andi %[rgb], %[rgb_011]   ; 1       // ...otherwise clear blue bit    \n"
        "pwm_data_out:                      ;                                           \n"
        "         out  %[port], %[rgb]      ; 1       // write rgb data to port (with SHCP = STCP = LO) \n"
        "pwm_clock_out:                     ;                                           \n"
        "         sbi  %[port], %[shcp_bit] ; 2       // SHCP hi on PORTD               \n"
        "         dec  %[col]               ; 1       // next column                    \n" 
        "         brne pwm_tick_loop        ; 1/2     // repeat if %[col] still > 0     \n"
        : [col]      "+r"  (col), // "r" means any register
                     "+e"  (rowData) // %a1
        : [tick]     "r"   (tick), 
          [port]     "I"   (_SFR_IO_ADDR(s595_DS_port)),
          [rgb]      "d"   (0),  // "d" means upper register
          [rgb_001]  "I"   (                                         _BV(s595_DS_bit) ),
          [rgb_010]  "I"   (                     _BV(s595_DSb_bit)                    ),
          [rgb_011]  "I"   (                     _BV(s595_DSb_bit) | _BV(s595_DS_bit) ),
          [rgb_100]  "I"   ( _BV(s595_DSc_bit)                                        ),
          [rgb_101]  "I"   ( _BV(s595_DSc_bit)                     | _BV(s595_DS_bit) ),
          [rgb_110]  "I"   ( _BV(s595_DSc_bit) | _BV(s595_DSb_bit)                    ),
          [rgb_111]  "I"   ( _BV(s595_DSc_bit) | _BV(s595_DSb_bit) | _BV(s595_DS_bit) ),
          [shcp_bit] "I"   (s595_SHCP_bit)
    );
#endif
// ----------------------------------------
#ifdef pwm_tick_C1
    volatile byte* rowData = (byte*)&data_rgb[pwm_row_index];
    byte tick = pwm_tick;
    byte col = pwm_columnCount;
    TCNT3 = 0;
    while (col-- != 0) {  // faster than for "(byte col = 0; col < pwm_columnCount; i++)"
      byte out = 0;
 
      if (*rowData++ > tick) {
        out |= (1 << s595_DS_bit);  // "bank" a
      }
      if (*rowData++ > tick) {  // "bank" b
        out |= (1 << s595_DSb_bit);
      }
      if (*rowData++ > tick) {  // "bank" c
        out |= (1 << s595_DSc_bit);
      }
      PORTD = out; // also sets SHCP and STCP low
      s595_SHCP_HI;
    }
#endif
// ----------------------------------------
#ifdef pwm_tick_C2
    volatile byte* rowData = (byte*)&data_rgb[pwm_row_index];
    byte tick = pwm_tick;
    byte col = pwm_columnCount;
    TCNT3 = 0;
    while (col-- != 0) {  // faster than for "(byte col = 0; col < pwm_columnCount; i++)"
      byte inA = *rowData++;
      byte inB = inA & 0x1F;
      if (inA > pwm_tick) {
        out |= (1 << s595_DS_bit);  // "bank" a
      }
      inB = *rowData++;
      if (inB > pwm_tickHI) {  // "bank" c
        out |= (1 << s595_DSc_bit);
      }
      inA >>= 2;
      inA |= inB << 6;
      if (inA > pwm_tickHI) {  // "bank" b
        out |= (1 << s595_DSb_bit);
      }
      PORTD = out; // also sets SHCP and STCP low
      s595_SHCP_HI;
   
    }
#endif

      uint16_t temp = TCNT3;
    if (timer_intr || ((_pwm_tick >= pwm_tick) && (_pwm_tick < pwm_tickCount - 1))) {
      missed++;
    } else {
      _missed = missed;
      missed = 0;
      _pwm_tick = pwm_tick;
      _tm_1 = temp;
      _tm_0 = TCNT3 + 16; // add 16 cycles for rest of fn plus returning
      timer_intr = true;
    }

  }


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
      uint16_t a      = _tm_0;
      uint16_t b      = _tm_1;
      uint16_t missed = _missed;
      uint8_t  tick   = _pwm_tick;
      timer_intr = false;
      Serial.print(a);
      Serial.print(" / ");
      Serial.print(b);
      Serial.print(" cycles (");
      Serial.print(a * 0.0625);
      Serial.print(" / ");
      Serial.print(b * 0.0625);
      Serial.print(" micros)");
      /*
      Serial.print(", s595_int: ");
      Serial.print(ltoa(((unsigned long)1 << pwm_columnCount) | s595_state_int, buf, 2));
      Serial.print(", s595_ext: ");
      Serial.print(ltoa(((unsigned long)1 << pwm_columnCount) | s595_state_ext, buf, 2));
      */
      Serial.print(", pwm_tick: ");
      Serial.print(tick);
      Serial.print(", missed: ");
      Serial.print(missed);
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




