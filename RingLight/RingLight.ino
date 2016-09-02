#include <Arduino.h>

// The pins used to communicate with the shift registers (74HC595)
#define enablePin  7 // active-low; aka OE
#define clearPin   6 // active-low; aka MR
#define latchPin   5 // active-high; aka STCP/ST_CP
#define clockPin   4 // active-high; aka SHCP/SH_CP
#define dataPin    3 // active-high; aka DS

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

#define MAX_BRIGHTNESS 32


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

/*
// =23/47/79
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
#define setBit(bit) asm volatile("sbi %[port], %[bitnr]   " : : [port] "I" (_SFR_IO_ADDR(PORTD)), [bitnr] "I" (bit))
#define clrBit(bit) asm volatile("cbi %[port], %[bitnr]   " : : [port] "I" (_SFR_IO_ADDR(PORTD)), [bitnr] "I" (bit))
#define pulseBitH(bit) setBit(bit); clrBit(bit);
#define pulseBitL(bit) clrBit(bit); setBit(bit);
#define pulse_clear() pulseBitL(clearBit) // falling edge here clears the shiftregs; leaves clearPin HIGH (it's active-low)
#define pulse_latch() pulseBitH(latchBit) // rising edge here transfers shifted-in data to outputs; leaves latchPin LOW
#define pulse_clock() pulseBitH(clockBit) // rising edge here shifts in the bit currently present at DS; leaves clockPin LOW
#define enable_on()  clrBit(enableBit) // active-low
#define enable_off() setBit(enableBit) // -"-
#define set_data()   setBit(dataBit)
#define clr_data()   clrBit(dataBit)


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
#define TIMER2_PRESCALE_BY_64   ((1 << CS22)                            ) // 250.000 MHz /   4.0 µs 
#define TIMER2_PRESCALE_BY_128  ((1 << CS22)               | (1 << CS20)) // 125.000 MHz /   8.0 µs
#define TIMER2_PRESCALE_BY_256  ((1 << CS22) | (1 << CS21)              ) //  62.500 KHz /  16.0 µs
#define TIMER2_PRESCALE_BY_1024 ((1 << CS22) | (1 << CS21) | (1 << CS20)) //  15.625 KHz /  64.0 µs

void configure_interrupts(void) {
  noInterrupts();
  
  // Timer 1 (16 bit)
  TCCR1A =  ((1 << WGM11) & 0x00) | ((1 << WGM10) & 0x00);                          // TIMER1 CTC mode ("Clear Timer on Compare")
  TCCR1B =  ((1 << WGM13) & 0x00) | ((1 << WGM12) & 0xFF) | TIMER1_PRESCALE_BY_1;   // TIMER1 CTC mode @ 16MHz
  TIMSK1 |= (1 << OCIE1A);  // enable compare interrupt for TIMER1
  OCR1A  = (5000) - 1;   // compare match register: interrupt every 312.5 µs ~> 3.2 KHz (32 brightness levels, refresh-rate 100Hz)
  TCNT1  = 0;
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
  digitalWrite(enablePin, HIGH);  // disable output (it's active-low)
  reset595();
  digitalWrite(enablePin, LOW); // re-enable output

  // put dataPin and clockPin in default state:
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);

  configure_interrupts();
  
  Serial.begin(57600);
}


#define CMD_NONE        0
#define CMD_UNKNOWN     1
#define CMD_EMPTY       2
#define CMD_BRIGHTNESS  3
#define CMD_ANIM        4
#define CMD_DELAY       5
#define CMD_INVERT      6
#define CMD_ROTATE      7
#define CMD_MIRROR      8

int arguments[3];

byte parseCommand() {
  byte c = CMD_UNKNOWN;
  int ch = Serial.read();
  switch (ch) {
    case -1:   
      return CMD_NONE;
    case '\n':
      return CMD_EMPTY;
    case 'b':
      c = CMD_BRIGHTNESS;
      ch = Serial.peek();
      arguments[0] = Serial.parseInt();
      break;
    case 'd':
      c = CMD_DELAY;
      arguments[0] = Serial.parseInt();
      break;
    case 'a':
      c = CMD_ANIM;
      arguments[0] = Serial.parseInt();
      break;
    case 'i':
      c = CMD_INVERT;
      while ((ch != -1) && (ch != '\n')) ch = Serial.read();
      break;
    case 'r':
      c = CMD_ROTATE;
      arguments[0] = Serial.parseInt();
      break;
    case 'm':
      c = CMD_MIRROR;
      while ((ch != -1) && (ch != '\n')) ch = Serial.read();
      break;
  }

  return c;
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

uint16_t anim_wanderingDot1_cc(uint16_t last, uint32_t ms) {
  last <<= 1;
  return (last == 0) ? 1 : last;
}

uint16_t anim_fountain(uint16_t last, uint32_t ms) {
  last <<= 1;
  last &= 0x01FF;
  if ((last & 0x0E) == 0) {
    last |= 1;
  }
  byte maskLo = 0x02;
  uint16_t maskHi = 0x8000;
  while (maskLo) {
    if (last & maskLo) {
      last |= maskHi;  
    }
    maskLo <<= 1;
    maskHi >>= 1;  
  }
  return last;
}

uint16_t anim_quarters(uint16_t last, uint32_t ms) {
  return last ? rotateLeft(last, 4) : 0x000F;
}

typedef uint16_t (*animFuncPtr)(uint16_t, uint32_t); 
volatile animFuncPtr animations[] = {
  anim_allOff,
  anim_ring,
  anim_wanderingDot1_cc,
  anim_fountain,
  anim_quarters,
};
#define animationCount (sizeof(animations)/sizeof(animFuncPtr))
volatile uint16_t animStates[animationCount];

volatile byte currentAnim = animationCount - 1;
volatile uint16_t brightness = 1;
volatile uint32_t animationDelay = 5; // in milliseconds
volatile bool     invert = false;
volatile byte     rotate = 0;
volatile bool     mirror = false;
volatile uint32_t animationTick = 0;
volatile uint16_t pwm_tick = MAX_BRIGHTNESS;
volatile uint16_t currentState;
volatile uint16_t currentMask;
uint16_t t_avg;
ISR(TIMER1_COMPA_vect) {
  uint16_t t;
  if (++pwm_tick >= MAX_BRIGHTNESS) {
    pwm_tick = 0;
    if (brightness == 0) {
      enable_off();
    } else {
      enable_on();
    }
    pulse_latch(); // transfer last state to outputs
    //Serial.println(t_avg /16.0);
    t_avg = 0;
    currentMask = 1 << 15; // initialize mask for next shifting cycle (pwm_ticks 1..16)

    if (++animationTick > animationDelay) { // Note: it's NOT >= here!
      uint16_t oldState = animStates[currentAnim];
      currentState = animations[currentAnim](oldState, 0);
      animStates[currentAnim] = currentState;
      if (invert) {
        currentState ^= 0xFFFF;  
      }
      currentState = rotateLeft(currentState, rotate);
      if (mirror) { // additional rotateLeft 1 st axis is vertical
        currentState = rotateLeft(mirrorBits(currentState), 1);
      }
      animationTick = 0;
    }

  } else {
    t = TCNT1;
    if (pwm_tick == brightness) {
      enable_off();
    }
    if (pwm_tick <= 16) {
      if (currentMask & currentState) {
        set_data();
      } else {
        clr_data();
      }
      pulse_clock();
      currentMask >>= 1;
      t_avg += TCNT1 - t;
    }
  }
}

bool serialConn = false;

void loop() {
  if (Serial) {
    if (!serialConn) { // greet if reconnected
      Serial.println("Hi there!");
      serialConn = true;
    }
    byte cmd = parseCommand();
    int a;
    switch (cmd) {
      case CMD_NONE:
      case CMD_EMPTY:
        break;
      case CMD_BRIGHTNESS:
        Serial.print("brightness: ");
        a = constrain(arguments[0], 0, MAX_BRIGHTNESS);
        Serial.println(a);
        brightness = a;
        break;
      case CMD_ANIM:
        a = arguments[0];
        if ((a < 0) || (a >= animationCount)) {
          Serial.print("no such animation: ");
          Serial.println(a);
        } else {
          currentAnim = a;
          Serial.print("animation: ");
          Serial.println(a);
        }
        break;
      case CMD_DELAY:
        a = arguments[0];
        if (a < 0) {
          a = 0;
        }
        animationDelay = a;
        Serial.print("delay: ");
        Serial.println(a);
        break;
      case CMD_INVERT:
        invert = !invert;
        Serial.print("invert: ");
        Serial.println(invert ? "on" : "off");
        break;
      case CMD_MIRROR:
        mirror = !mirror;
        Serial.print("mirror: ");
        Serial.println(mirror ? "on" : "off");
        break;
      case CMD_ROTATE:
        a = arguments[0];
        rotate = a & 0x0F;
        Serial.print("rotate: ");
        Serial.println(rotate);
        break;

      default:
        Serial.print("unknown command ");
        Serial.println(cmd);
    }
  } else {
    serialConn = false;  
  }

  delay(1);
}


