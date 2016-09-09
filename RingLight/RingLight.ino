#include "Arduino.h"  // could use <angle brackets> instead of "double quotes"
#include "Command.h"  // MUST use "double quotes" for this
#include "EEPROM.h"   // could use <angle brackets> instead of "double quotes"

// The pins used to communicate with the shift registers (74HC595)
#define enablePin  3 // connect OE (which is active-low) of 74HC595 to collector of an NPN, and pin 3 with a 10K to the base (emitter to GND)
#define clearPin   6 // active-low; aka MR
#define latchPin   5 // active-high; aka STCP/ST_CP
#define clockPin   4 // active-high; aka SHCP/SH_CP
#define dataPin    7 // active-high; aka DS

#define enableBit enablePin
#define clearBit  clearPin
#define latchBit  latchPin
#define clockBit  clockPin
#define dataBit   dataPin

// You can connect LEDs (with resistor) between the above 
// pins and GND to see what's going on. But in order to be
// able to see anything useful, things need to be slowed down:
//#define visible_signals
#define signal_duration_ms 50

#define MAX_BRIGHTNESS          15  // must be odd!
#define BRIGHTNESS_LEVELS       (MAX_BRIGHTNESS + 1)
#define TIMER1_TOP_DELTA        (MAX_BRIGHTNESS * 2)
#define TIMER1_TOP_MIN          (90-1) // @2MHz (PRESCALE_BY_8) this gives 2000/(TIMER1_TOP+1) KHz PWM frequency
#define TIMER1_TOP_MAX          (TIMER1_TOP_MIN + TIMER1_TOP_DELTA)
#define TIMER1_TOP              ((TIMER1_TOP_MIN + TIMER1_TOP_MAX) >> 1) // center between TIMER1_TOP_MIN and TIMER1_TOP_MAX
#define DISPLAY_COLUMNS         (4*8)
#define DISPLAY_ROWS            8
volatile byte rowCount = DISPLAY_ROWS;

void pulseXXX(int pin, bool active_high) {
#ifdef visible_signals
  delay(signal_duration_ms);
#endif
  digitalWrite(pin,  active_high); // true  == HIGH
#ifdef visible_signals
  delay(signal_duration_ms);
#endif
  digitalWrite(pin, !active_high); // false == LOW
#ifdef visible_signals
  delay(signal_duration_ms);
#endif
}

/*
// =225/318/334
#define setPin(pin) digitalWrite(pin, HIGH)
#define clrPin(pin) digitalWrite(pin, LOW)
#define pulseH(pin) pulseXXX(pin, HIGH);
#define pulseL(pin) pulseXXX(pin, LOW);
#define pulse_clear() pulseL(clearPin) // falling edge here clears the shiftregs; leaves clearPin HIGH (it's active-low)
#define pulse_latch() pulseH(latchPin) // rising edge here transfers shifted-in data to outputs; leaves latchPin LOW
#define pulse_clock() pulseH(clockPin) // rising edge here shifts in the bit currently present at DS; leaves clockPin LOW
#define enable_on()  clrPin(enablePin) // active-low
#define enable_off() setPin(enablePin) // -"-
#define set_data()   setPin(dataPin)
#define clr_data()   clrPin(dataPin)
*/


/*
// =64/107/140
#define setPin(pin) *(portOutputRegister(digitalPinToPort(pin))) |=  (byte)digitalPinToBitMask(pin)
#define clrPin(pin) *(portOutputRegister(digitalPinToPort(pin))) &= ~(byte)digitalPinToBitMask(pin)
#define pulseH(pin) setPin(pin); clrPin(pin)
#define pulseL(pin) clrPin(pin); setPin(pin)
#define pulse_clear() pulseL(clearPin) // falling edge here clears the shiftregs; leaves clearPin HIGH (it's active-low)
#define pulse_latch() pulseH(latchPin) // rising edge here transfers shifted-in data to outputs; leaves latchPin LOW
#define pulse_clock() pulseH(clockPin) // rising edge here shifts in the bit currently present at DS; leaves clockPin LOW
#define enable_on()  clrPin(enablePin) // active-low
#define enable_off() setPin(enablePin) // -"-
#define set_data()   setPin(dataPin)
#define clr_data()   clrPin(dataPin)
*/


/*// =23/47/79
#define setPin(pin) PORTD |=  (byte)digitalPinToBitMask(pin)
#define clrPin(pin) PORTD &= ~(byte)digitalPinToBitMask(pin)
#define pulseH(pin) setPin(pin); clrPin(pin)
#define pulseL(pin) clrPin(pin); setPin(pin)
#define pulse_clear() pulseL(clearPin) // falling edge here clears the shiftregs; leaves clearPin HIGH (it's active-low)
#define pulse_latch() pulseH(latchPin) // rising edge here transfers shifted-in data to outputs; leaves latchPin LOW
#define pulse_clock() pulseH(clockPin) // rising edge here shifts in the bit currently present at DS; leaves clockPin LOW
#define enable_on()  clrPin(enablePin) // active-low
#define enable_off() setPin(enablePin) // -"-
#define set_data()   setPin(dataPin)
#define clr_data()   clrPin(dataPin)
*/


// =8/21/53
#define setBit(bitNr)     asm volatile("sbi %[port], %[bitnr]   " : : [port] "I" (_SFR_IO_ADDR(PORTD)), [bitnr] "I" (bitNr))
#define clrBit(bitNr)     asm volatile("cbi %[port], %[bitnr]   " : : [port] "I" (_SFR_IO_ADDR(PORTD)), [bitnr] "I" (bitNr))
#define pulseBitH(bitNr)  setBit(bitNr); clrBit(bitNr)
#define pulseBitL(bitNr)  clrBit(bitNr); setBit(bitNr)
#define pulse_clear()     pulseBitL(clearBit) // falling edge here clears the shiftregs; leaves clearPin HIGH (it's active-low)
#define pulse_latch()     pulseBitH(latchBit) // rising edge here transfers shifted-in data to outputs; leaves latchPin LOW
#define pulse_clock()     pulseBitH(clockBit) // rising edge here shifts in the bit currently present at DS; leaves clockPin LOW
#define enable_on()       setBit(enableBit) // active-hi
#define enable_off()      clrBit(enableBit) // -"-
#define set_data()        setBit(dataBit)
#define clr_data()        clrBit(dataBit)


void reset595() {
  pulse_clear();  // falling edge here clears the shiftregs; leaves clearPin HIGH (it's active-low)
  pulse_latch();  // rising edge here transfers shifted-in data to outputs; leaves latchPin LOW
}


#define TIMER1_PRESCALE_STOPPED 0
#define TIMER1_PRESCALE_BY_1    (                       _BV(CS10))  //  16.000 MHz /  62.5 ns
#define TIMER1_PRESCALE_BY_8    (           _BV(CS11)            )  //   2.000 MHz / 500.0 ns
#define TIMER1_PRESCALE_BY_64   (           _BV(CS11) | _BV(CS10))  // 250.000 KHz /   4.0 µs
#define TIMER1_PRESCALE_BY_256  (_BV(CS12)                       )  //  62.500 KHz /  16.0 µs
#define TIMER1_PRESCALE_BY_1024 (_BV(CS12)            | _BV(CS10))  //  15.625 KHz /  64.0 µs

#define TIMER2_PRESCALE_STOPPED 0
#define TIMER2_PRESCALE_BY_1    (                            (1 << CS20)) //  16.000 MHz /  62.5 ns
#define TIMER2_PRESCALE_BY_8    (              (1 << CS21)              ) //   2.000 MHz / 500.0 ns
#define TIMER2_PRESCALE_BY_32   (              (1 << CS21) | (1 << CS20)) // 500.000 KHz /   2.0 µs
#define TIMER2_PRESCALE_BY_64   ((1 << CS22)                            ) // 250.000 KHz /   4.0 µs 
#define TIMER2_PRESCALE_BY_128  ((1 << CS22)               | (1 << CS20)) // 125.000 KHz /   8.0 µs
#define TIMER2_PRESCALE_BY_256  ((1 << CS22) | (1 << CS21)              ) //  62.500 KHz /  16.0 µs
#define TIMER2_PRESCALE_BY_1024 ((1 << CS22) | (1 << CS21) | (1 << CS20)) //  15.625 KHz /  64.0 µs

void configure_interrupts(void) {
  noInterrupts();
  
  // Timer 1 (16 bit)
  TCCR1A =  ((1 << WGM11) & 0x00) | ((1 << WGM10) & 0x00);                          // TIMER1 CTC mode ("Clear Timer on Compare")
  TCCR1B =  ((1 << WGM13) & 0x00) | ((1 << WGM12) & 0xFF) | TIMER1_PRESCALE_BY_8;   // TIMER1 CTC mode @ 2MHz
  OCR1A  = TIMER1_TOP_MIN;   // compare match register: interrupt every (TIMER1_TOP+1)/2 µs ~> 2000/(TIMER1_TOP+1) KHz
  TCNT1  = 0;
  TIMSK1 |= (1 << OCIE1A);  // enable compare interrupt for TIMER1
/* there is no TIMER2 on the Atmega32uXX
  TCCR2 = 0 | TIMER2_PRESCALE_BY_8; // TIMER2 in normal mode @ 2MHz / 0.5µs period (overflow after 128 µs)
  TIMSK2 &= ~(_BV(TOIE2) | _BV(OCIE2A) | _BV(OCIE2B)); // no interrupts from TIMER2
*/ 
  interrupts();
}

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(clearPin,  OUTPUT);
  pinMode(latchPin,  OUTPUT);
  pinMode(clockPin,  OUTPUT);
  pinMode(dataPin,   OUTPUT);

  // clear shift regs first:
  
  digitalWrite(enablePin, LOW);  // disable output
  reset595();
  digitalWrite(enablePin, HIGH); // re-enable output

  // put dataPin and clockPin in default state:
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);

  configure_interrupts();
  
  Serial.begin(57600);
  Serial.println("setup done");
}

volatile uint16_t brightness = 1;
volatile uint32_t animationDelay = 5; // in milliseconds
volatile bool     invert = false;
volatile byte     rotate = 0;
volatile bool     mirror = false; // mirror at vertical axis (does not depend on rotate)
volatile bool     flip = false;   // mirror at 0-axis (depends on rotate)



enum { // node type
  TYPE_CMD   = 0,
  TYPE_INT   = 1,
  TYPE_DELTA = 2
};

#define CMD_NONE        0
#define CMD_UNKNOWN     1
#define CMD_EMPTY       2
#define CMD_BRIGHTNESS  3
#define CMD_ANIM        4
#define CMD_DELAY       5
#define CMD_INVERT      6
#define CMD_ROTATE      7
#define CMD_MIRROR      8
#define CMD_FLIP        9
#define CMD_TIME       10
#define CMD_ROWCOUNT   11
#define CMD_EEPROM     12



cmd_t cmd_invert = cmd_t(
  CMD_INVERT,       // id  
  'i',              // chr
  "invert",         // name
  &invert           // valuePtr
);
cmd_t cmd_flip = {
  CMD_FLIP,         // id  
  'f',              // chr
  "flip",           // name
  &flip             // valuePtr
};
cmd_t cmd_mirror = {
  CMD_MIRROR,       // id  
  'm',              // chr
  "mirror",         // name
  &mirror           // valuePtr
};
cmd_t cmd_rotate = cmd_t(
  CMD_ROTATE,       // id  
  'r',              // chr
  "rotate",         // name
  &rotate,          // valuePtr
  0, 15,            // min, max
  CMDFLAG_WRAPAROUND  // additional flags
);

void dumpEEPROM() {
    const byte bytes_per_line = 16;
    const char unprintable_replacement = 0x90;
    char sbuf[4 + 2 + 3*bytes_per_line + 2 + bytes_per_line + 1];
    uint16_t n = EEPROM.length();
    uint16_t i = 0;
    /*
    for (i = 0; i < 256; i++) {
      EEPROM.put(i, (byte)i);
    }
    */
    snprintf(sbuf, sizeof(sbuf), "%d bytes of EEPROM:", n);
    Serial.println(sbuf);
    while (i < n) {
      snprintf(sbuf, sizeof(sbuf), "%04X: ", i);
      byte j = 5;
      byte k = 5 + bytes_per_line*3 + 2;
      for (byte x = bytes_per_line; (x > 0) && (i < n); x--) {
        byte v = EEPROM[i++];
        snprintf(&sbuf[j], 4, " %02X", v);
        j += 3;
        sbuf[k++] = isPrintable(v) ? v : unprintable_replacement;
      }
      sbuf[j]   = ' '; // overwrite the zero byte from snprintf (after last hex byte in line)
      sbuf[j+1] = ' '; // put another space between hex and ascii values
      sbuf[k]   = 0;   // terminate the whole line string
      Serial.println(sbuf);
    }  
}

cmd_t cmd_eeprom = cmd_t(
  CMD_EEPROM,       // id  
  'e',              // chr
  "EEPROM dump",    // name
  &dumpEEPROM
);

struct node_t {
  byte type;
  union {
    int i;
    char c;
  } value;
  byte childCount;
  struct nodeList_t *children;
};

struct nodeList_t {
  node_t node;
  struct nodeList_t *next;  
};

char parsedChars[10];
byte parsedCharCount;

#define STATE_LINE_START    0
#define STATE_LINE_END      1
#define STATE_OPTIONAL_ARG  2
#define STATE_NEXT_DIGIT    3
#define STATE_SKIP_TIL_EOL  4
#define STATE_ERROR         5

struct node_t *parseCommand() {
  static byte state = STATE_LINE_START;
  static nodeList_t children = { { TYPE_INT, 0, 0, NULL }, NULL };
  static node_t ast = { TYPE_CMD, CMD_UNKNOWN, 0, &children };
  static node_t *child = &(children.node);
  static int temp = 0;
  int ch;
  do {
    ch = Serial.read();
    if (ch == -1) {     // timeout, stay in same state
      return NULL;      // and report nothing
    }
    if (state == STATE_LINE_START) {
      parsedCharCount = 0;
      parsedChars[0] = 0;  
    }
    if (ch != '\n') {
      parsedCharCount++;
      if (parsedCharCount < sizeof(parsedChars)) {
        parsedChars[parsedCharCount - 1] = (byte)ch;
        parsedChars[parsedCharCount] = 0;
      }
    }
    switch (state) {
      case STATE_SKIP_TIL_EOL:
        if (ch == '\n') {
          state = STATE_LINE_END;  
        }
        break;
      case STATE_NEXT_DIGIT:
        switch (ch) {
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            temp *= 10;
            temp += ch - '0';
            break;
          default:
            ast.childCount++;
            child->type = TYPE_INT;
            child->value.i = temp;
            state = (ch = '\n') ? STATE_LINE_END : STATE_SKIP_TIL_EOL;
        }
        break;
      case STATE_OPTIONAL_ARG:
        switch (ch) {
          case '\n':
            state = STATE_LINE_END;
            break;
          case ' ':   // ignore whitespace
          case '\t':
            break;
          case '+':
            ast.childCount++;
            child->type = TYPE_DELTA;
            child->value.i = 1;
            state = STATE_SKIP_TIL_EOL;
            break;
          case '-':
            ast.childCount++;
            child->type = TYPE_DELTA;
            child->value.i = -1;
            state = STATE_SKIP_TIL_EOL;
            break;
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            temp = ch - '0';
            state = STATE_NEXT_DIGIT;
            break;
          default:
            state = STATE_ERROR;
        }
        break;
      case STATE_LINE_START:
        if (ch == '\n') {
          state = STATE_LINE_END;
        } else {
          ast.type = TYPE_CMD;
          ast.childCount = 0;
          switch (ch) {
            case 'b':
              ast.value.c = CMD_BRIGHTNESS;
              state = STATE_OPTIONAL_ARG;
              break;
            case 'p':
              ast.value.c = CMD_ROWCOUNT;
              state = STATE_OPTIONAL_ARG;
              break;
            case 'd':
              ast.value.c = CMD_DELAY;
              state = STATE_OPTIONAL_ARG;
              break;
            case 'a':
              ast.value.c = CMD_ANIM;
              state = STATE_OPTIONAL_ARG;
              break;
            case 'r':
              ast.value.c = CMD_ROTATE;
              state = STATE_OPTIONAL_ARG;
              break;
            case 'i':
              ast.value.c = CMD_INVERT;
              state = STATE_SKIP_TIL_EOL;
              break;
            case 'm':
              ast.value.c = CMD_MIRROR;
              state = STATE_SKIP_TIL_EOL;
              break;
            case 'f':
              ast.value.c = CMD_FLIP;
              state = STATE_SKIP_TIL_EOL;
              break;
            case 'e':
              ast.value.c = CMD_EEPROM;
              state = STATE_SKIP_TIL_EOL;
              break;
            case 't':
              ast.value.c = CMD_TIME;
              state = STATE_SKIP_TIL_EOL;
              break;
            default:
              state = STATE_ERROR;
          }
        }
        break;  // STATE_LINE_START
    } // switch(state)
    if (state == STATE_ERROR) {
      ast.value.c = CMD_UNKNOWN;
      state = STATE_SKIP_TIL_EOL;
    }
  } while (state != STATE_LINE_END);
  state = STATE_LINE_START;
  return &ast;
}

uint16_t rotateLeft(uint16_t x, byte n) {
  n &= 0x0F;
  while (n) {
    bool one = x & 0x8000;
    x <<= 1;
    if (one) {
      x |= 1;  
    }
    n--;
  }
  return x;
}

uint16_t rotateRight(uint16_t x, byte n) {
  return rotateLeft(x, (16 - (n & 0xF)) & 0xF);
}

uint16_t mirrorBits(uint16_t x) {
  uint16_t result = 0;
  uint16_t mask = 0x8000;
  while (x) {
    if (x & 1) {
      result |= mask;  
    }
    x >>= 1;
    mask >>= 1;
  }
  return result;
}

uint16_t anim_allOff(uint16_t last, uint32_t ms) {
  return 0;
}

uint16_t anim_ring(uint16_t last, uint32_t ms) {
  if ((last == 0) || (~last == 0)) {
    return 1;
  } else {
    return (last << 1) | 1;
  }
}

uint16_t anim_wanderingRing(uint16_t last, uint32_t ms) {
  static byte rotation = 0; // nicer effects if we were to change the global var rotate...
  if (~last == 0) {
    rotation = (rotation + 1) & 0xF;  
  }
  last = anim_ring(rotateRight(last, rotation), ms);
  last = rotateLeft(last, rotation);
  return last;
}

uint16_t anim_wanderingDot1(uint16_t last, uint32_t ms) {
  last <<= 1;
  return (last == 0) ? 1 : last;
}

uint16_t anim_fountain(uint16_t last, uint32_t ms) {
  last <<= 1;
  last &= 0x01FF;
  if ((last & 0x0E) == 0) {
    last |= 1;
  }
  return last | (mirrorBits(last) << 1);
}

uint16_t anim_quarters(uint16_t last, uint32_t ms) {
  return last ? rotateLeft(last, 4) : 0x000F;
}

uint16_t anim_binaryUpCounter(uint16_t last, uint32_t ms) {
  return last + 1;
}

uint16_t anim_clock(uint16_t last, uint32_t ms) {
  uint16_t result = 0;
  uint16_t secondsEighths = ms / 125;
  uint16_t minutes16th = ms / 3750;
  if ((ms / 500) & 1) { // have seconds indicator blink at 1Hz
    result |= 1 << (minutes16th & 0x000F);  
  }
  uint16_t hours16th = minutes16th / 60;
  if (secondsEighths & 0xF) {
    result ^= 1 << (hours16th & 0x000F); // minutes indicator
  }
  /*
  Serial.print(ms);
  Serial.print("\t");
  Serial.println(minutes16th);
  */
  return rotateLeft(mirrorBits(result), 1);
}

typedef uint16_t (*animFuncPtr)(uint16_t, uint32_t); 
volatile animFuncPtr animations[] = {
  anim_allOff,
  anim_ring,
  anim_wanderingRing,
  anim_wanderingDot1,
  anim_fountain,
  anim_quarters,
  anim_binaryUpCounter,
  anim_clock,
};
#define animationCount (sizeof(animations)/sizeof(animFuncPtr))
volatile uint16_t animStates[animationCount];

volatile byte     currentAnim = animationCount - 1;
volatile uint32_t milliseconds = 0;
volatile bool     milliseconds_req = false;

volatile uint16_t timer1_time_avg;
volatile uint16_t timer1_time_min;
volatile uint16_t timer1_time_max;
volatile bool     timer1_time_req = false;

volatile bool     pending_displayEvent = false;
volatile uint16_t missed_displayEvents = 0;

typedef volatile byte displayBuffer_t[DISPLAY_COLUMNS*DISPLAY_ROWS];
displayBuffer_t displayBufferA, displayBufferB;

typedef volatile byte* displayBufferPtr_t;
displayBufferPtr_t frontBuffer = displayBufferA;
displayBufferPtr_t backBuffer  = displayBufferB;

volatile uint16_t pwm_durations[BRIGHTNESS_LEVELS] = {
  TIMER1_TOP - ((TIMER1_TOP_DELTA * 8) / BRIGHTNESS_LEVELS),
  TIMER1_TOP - ((TIMER1_TOP_DELTA * 7) / BRIGHTNESS_LEVELS),
  TIMER1_TOP - ((TIMER1_TOP_DELTA * 6) / BRIGHTNESS_LEVELS),
  TIMER1_TOP - ((TIMER1_TOP_DELTA * 5) / BRIGHTNESS_LEVELS),
  TIMER1_TOP - ((TIMER1_TOP_DELTA * 4) / BRIGHTNESS_LEVELS),
  TIMER1_TOP - ((TIMER1_TOP_DELTA * 3) / BRIGHTNESS_LEVELS),
  TIMER1_TOP - ((TIMER1_TOP_DELTA * 2) / BRIGHTNESS_LEVELS),
  TIMER1_TOP - ((TIMER1_TOP_DELTA * 1) / BRIGHTNESS_LEVELS),
  // <--- center
  TIMER1_TOP + ((TIMER1_TOP_DELTA * 1) / BRIGHTNESS_LEVELS),
  TIMER1_TOP + ((TIMER1_TOP_DELTA * 2) / BRIGHTNESS_LEVELS),
  TIMER1_TOP + ((TIMER1_TOP_DELTA * 3) / BRIGHTNESS_LEVELS),
  TIMER1_TOP + ((TIMER1_TOP_DELTA * 4) / BRIGHTNESS_LEVELS),
  TIMER1_TOP + ((TIMER1_TOP_DELTA * 5) / BRIGHTNESS_LEVELS),
  TIMER1_TOP + ((TIMER1_TOP_DELTA * 6) / BRIGHTNESS_LEVELS),
  TIMER1_TOP + ((TIMER1_TOP_DELTA * 7) / BRIGHTNESS_LEVELS),
  TIMER1_TOP + ((TIMER1_TOP_DELTA * 8) / BRIGHTNESS_LEVELS),
};

#define switch_row(row) enable_off(); /* TODO: actually switch to next row */ pulse_latch(); enable_on()

ISR(TIMER1_COMPA_vect) {
  static uint16_t halfMicros = 0; // assuming TIMER1 @2MHz (PRESCALE_BY_8)
  static uint32_t milliseconds_private = 0;
  static uint16_t pwm_tick = MAX_BRIGHTNESS;
  static byte     row = 1; // ranges from 1..rowCount
  static uint16_t t_min_private = 0xFFFF;
  static uint32_t t_avg_private = 0;
  static uint16_t t_max_private = 0;

  halfMicros += (OCR1A+1);
  if (halfMicros >= 2000) {
    halfMicros -= 2000;
    milliseconds_private++;
  }
  if (milliseconds_req) {
    milliseconds = milliseconds_private;
    milliseconds_req = false;
  }
  
  if (++pwm_tick >= MAX_BRIGHTNESS) {
    if (--row == 0) {
      row = rowCount;
      if (!pending_displayEvent) {
        displayBufferPtr_t temp = frontBuffer;
        frontBuffer = backBuffer;
        backBuffer = temp;
        milliseconds = milliseconds_private;
        pending_displayEvent = true;
      } else {
        missed_displayEvents++;
      }
    }
    if (timer1_time_req) {
      timer1_time_avg = t_avg_private / MAX_BRIGHTNESS;
      timer1_time_min = t_min_private;
      timer1_time_max = t_max_private;
      timer1_time_req = false;
    }
    t_avg_private = 0;
    //t_min_private = 0xFFFF;
    //t_max_private = 0;
    pwm_tick = 0;
  }
  if (row > DISPLAY_ROWS) { // rows above DISPLAY_ROWS are "virtual", for reducing the overall brightness
    enable_off();
  } else {
    byte i = DISPLAY_COLUMNS;
    displayBufferPtr_t rowEnd = frontBuffer + (row * DISPLAY_COLUMNS);
    asm volatile("                            ; cycles  // comment                        \n"
        "         rjmp pwm_tick_loop0         ; 2       // assuming data_bit is 0 initially \n"
        "pwm_zeroBit:                                                                     \n"
        "         cbi  %[port], %[data_bit]   ; 2       // clear data bit                 \n"
        "         sbi  %[port], %[clock_bit]  ; 2       // SHCP HI                        \n"
        "         dec  %[i]                   ; 1       // next?                          \n" 
        "         breq pwm_tick_end           ; 1/2     // stop if i == 0                 \n"
        "pwm_tick_loop0:                      ; -       // last data bit was a 0          \n"
        "         cbi  %[port], %[clock_bit]  ; 2       // SHCP LO (bogus on first iteration - doesn't matter) \n"
        "         ld   __tmp_reg__, -%a1      ; 2       // fetch value                    \n" 
        "         cp   %[tick], __tmp_reg__   ; 1       // tick < value ?                 \n"
        "         brlo pwm_oneBit             ; 1/2     // if so shift out a 1            \n"
        "         sbi  %[port], %[clock_bit]  ; 2       // else data bit is still 0 -> SHCP HI \n"
        "         dec  %[i]                   ; 1       // next?                          \n" 
        "         brne pwm_tick_loop0         ; 1/2     // repeat if %[i] still > 0       \n"
        "         rjmp pwm_tick_end           ; 2                                         \n"
        "pwm_oneBit:                                                                      \n"
        "         sbi  %[port], %[data_bit]   ; 2                                         \n"
        "         sbi  %[port], %[clock_bit]  ; 2       // SHCP HI                        \n"
        "         dec  %[i]                   ; 1       // next?                          \n" 
        "         breq pwm_tick_end_clear_data; 2       // stop if i==0                   \n"
        "pwm_tick_loop1:                      ; -       // last data bit was a 1          \n"
        "         cbi  %[port], %[clock_bit]  ; 2       // SHCP LO                        \n"
        "         ld   __tmp_reg__, -%a1      ; 2       // fetch value                    \n" 
        "         cp   %[tick], __tmp_reg__   ; 1       // tick >= value ?                \n"
        "         brsh pwm_zeroBit            ; 1/2     // if so shift out a 0            \n"
        "         sbi  %[port], %[clock_bit]  ; 2       // else data bit is still 1 -> SHCP HI \n"
        "         dec  %[i]                   ; 1       // next?                          \n" 
        "         brne pwm_tick_loop1         ; 1/2     // repeat if %[i] still > 0       \n"
        "pwm_tick_end_clear_data:                                                         \n"
        "         cbi  %[port], %[data_bit]   ; 2       // ensure data bit == 0 in the end  \n"
        "pwm_tick_end:                                                                    \n"
        "         cbi  %[port], %[clock_bit]  ; 2       // SHCP LO                        \n"
        : [i]         "+r"  (i), // "r" means any register
                      "+e"  (rowEnd) // %a1
        : [tick]      "r"   (pwm_tick), 
          [port]      "I"   (_SFR_IO_ADDR(PORTD)),
          [data_bit]  "I"   (dataBit),
          [clock_bit] "I"   (clockBit)
    );
    switch_row(row);  // latch pin is pulsed in here
  }
  
  TIMSK1 &= ~(1 << OCIE1A);  // disable compare interrupt for TIMER1
  //t_avg_private += TCNT1;
  OCR1A = pwm_durations[pwm_tick];
  TIMSK1 |= (1 << OCIE1A);  // enable compare interrupt for TIMER1

  uint16_t t = TCNT1;
  
  t_avg_private += t;
  t_min_private = min(t, t_min_private);
  t_max_private = max(t, t_max_private);
}


void bitsToBuf(uint16_t bits, volatile byte *buf, byte bitCount) {
  bitCount--;
  uint16_t mask = 1 << bitCount;
  buf += bitCount;
  while (mask) {
    *buf-- = (bits & mask) ? brightness : 0;
    mask >>= 1;
  }
}

void doAnimations(uint32_t ms, volatile byte *buf) {
    static uint32_t animationTick = 0;
    if (++animationTick > animationDelay) { // Note: it's NOT >= here!
      uint16_t oldState = animStates[currentAnim];
      uint16_t currentState = animations[currentAnim](oldState, ms);
      animStates[currentAnim] = currentState;
      if (invert) {
        currentState ^= 0xFFFF;  
      }
      if (flip) {
        currentState = rotateLeft(mirrorBits(currentState), 1); // add. rotL s.t. axis goes through an LED (rather than between two LEDs)
      }
      currentState = rotateLeft(currentState, rotate);
      if (mirror) { // additional rotateLeft 1: see above
        currentState = rotateLeft(mirrorBits(currentState), 1);
      }
      animationTick = 0;
      bitsToBuf(currentState, buf, 16);
      bitsToBuf(brightness, buf + 16, 8);
    } 
}

void check_displayEvent() {
  if (pending_displayEvent) {
    memcpy((void*)backBuffer, (void*)frontBuffer, sizeof(displayBuffer_t));
    doAnimations(milliseconds, backBuffer);
    pending_displayEvent = false;
  }  
}

uint32_t millis2() {
  milliseconds_req = true;
  while (milliseconds_req) {
  }
  return milliseconds;  
}

void tm_timer1(uint16_t *min, uint16_t *avg, uint16_t *max) {
  timer1_time_req = true;
  while (timer1_time_req) {
  }
  *min = timer1_time_min;
  *avg = timer1_time_avg;
  *max = timer1_time_max;
}

void doCommands() {
  node_t *ast = parseCommand();
  if (ast != NULL) {
    int n = ast->childCount;
    node_t *a0 = (n > 0) ? &(ast->children->node) : NULL;
    uint32_t t1, t2;
    uint16_t tmin, tavg, tmax;
    switch (ast->value.c) {
      case CMD_BRIGHTNESS:
        if (a0) {
          if (a0->type == TYPE_INT) {
            brightness = a0->value.i;
          } else {
            brightness += a0->value.i;
          }
          brightness = constrain(brightness, 0, MAX_BRIGHTNESS);
          //analogWrite(enablePin, brightness);
        }
        Serial.print("brightness: ");
        Serial.println(brightness);
        break;
      case CMD_ROWCOUNT:
        if (a0) {
          if (a0->type == TYPE_INT) {
            rowCount = a0->value.i;
          } else {
            rowCount += a0->value.i;
          }
          rowCount = constrain(rowCount, 1, 256);
        }
        Serial.print("rowCount: ");
        Serial.println(rowCount);
        break;
      case CMD_ANIM:
        if (a0) {
          if (a0->type == TYPE_INT) {
            currentAnim = a0->value.i;
          } else {
            currentAnim += a0->value.i;
          }
          currentAnim = constrain(currentAnim, 0, (int)(animationCount - 1));
        }
        Serial.print("animation: ");
        Serial.println(currentAnim);
        break;
      case CMD_DELAY:
        if (a0) {
          if (a0->type == TYPE_INT) {
            animationDelay = a0->value.i;
          } else {
            animationDelay += a0->value.i;
          }
        }
        Serial.print("delay: ");
        Serial.print(animationDelay);
        Serial.print(" (");
        Serial.print(2000000.0 / ((TIMER1_TOP+1) * MAX_BRIGHTNESS * DISPLAY_ROWS * (animationDelay+1)));
        Serial.println(" Hz)");
        break;
      case CMD_ROTATE:
        if (a0) {
          if (a0->type == TYPE_INT) {
            cmd_rotate.set(a0->value.i);
          } else {
            cmd_rotate.inc(a0->value.i);
          }
        }
        cmd_rotate.report();
        break;
      case CMD_INVERT:
        cmd_invert.toggle()->report();
        break;
      case CMD_MIRROR:
        cmd_mirror.toggle()->report();
        break;
      case CMD_FLIP:
        cmd_flip.toggle()->report();
        //cmd_flip.report();
        break;
      case CMD_EEPROM:
        cmd_eeprom();
        break;
      case CMD_TIME:
        t1 = millis();
        t2 = millis2();
        tm_timer1(&tmin, &tavg, &tmax);
        Serial.print("time: ");
        Serial.print(t1);
        Serial.println(" ms");
        Serial.print("      ");
        Serial.println(t2);
        Serial.print("      ");
        Serial.println((int32_t)(t2 - t1));
        Serial.print("t_min: "); Serial.println(tmin);
        Serial.print("t_avg: "); Serial.println(tavg);
        Serial.print("t_max: "); Serial.println(tmax);
        Serial.print("missed: ");
        Serial.println(missed_displayEvents);
        break;

      default:
        Serial.print("unknown/invalid command \"");
        Serial.print(parsedChars);
        Serial.println("\"...");
    }
  } 
}

bool serialConn = false;

void loop() {
  if (Serial) {
    if (!serialConn) { // greet if reconnected
      Serial.print("TI1_TOP: "); Serial.print(TIMER1_TOP_MIN); Serial.print(".."); Serial.println(TIMER1_TOP_MAX);
      float f = 2000000.0/(TIMER1_TOP+1);
      Serial.print(" f_PWM: "); Serial.print(f); Serial.println(" Hz");
      f /= BRIGHTNESS_LEVELS;
      Serial.print("f_line:  "); Serial.print(f); Serial.print(" Hz ("); 
      Serial.print(DISPLAY_COLUMNS); Serial.print(" columns, ");
      Serial.print(BRIGHTNESS_LEVELS); Serial.println(" brightness lvls)");
      f = f / DISPLAY_ROWS;
      Serial.print("f_refr:   "); Serial.print(f); Serial.print(" Hz ("); Serial.print(DISPLAY_ROWS); Serial.println(" rows)");
      serialConn = true;
    }
    check_displayEvent();
    doCommands();
  } else {
    serialConn = false;  
  }


  delay(1);
}


