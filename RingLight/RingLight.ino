#include <Arduino.h>

// The pins used to communicate with the shift registers (74HC595)
#define enablePin  9 // active-low; aka OE
#define clearPin   8 // active-low; aka MR
#define latchPin   7 // active-high; aka STCP/ST_CP
#define clockPin   6 // active-high; aka SHCP/SH_CP
#define dataPin    5 // active-high; aka DS

// You can connect LEDs (with resistor) between the above 
// pins and GND to see what's going on. But in order to be
// able to see anything useful, things need to be slowed down:
//#define visible_signals
#define signal_duration_ms 50

#define MAX_BRIGHTNESS 32

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
  OCR1A  = (5000) - 1;   // compare match register: interrupt every 31.25 µs ~> 32 KHz (32 brightness levels, refresh-rate 100Hz)
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

void pulse(int pin, bool active_high) {
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

void reset595() {
  pulse(clearPin, LOW);   // falling edge here clears the shiftregs; leaves clearPin HIGH (it's active-low)
  pulse(latchPin, HIGH);  // rising edge here transfers shifted-in data to outputs; leaves latchPin LOW
}

#define CMD_NONE        0
#define CMD_UNKNOWN     1
#define CMD_EMPTY       2
#define CMD_BRIGHTNESS  3
#define CMD_ANIM        4
#define CMD_DELAY       5

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
  }

  return c;
}


uint16_t anim_ring_cw(uint16_t last, uint32_t ms) {
  if ((last == 0) || (~last == 0)) {
    return 1;
  } else {
    return (last << 1) | 1;
  }
}

uint16_t anim_ring_cc(uint16_t last, uint32_t ms) {
  if ((last == 0) || (~last == 0)) {
    return 1;
  } else {
    return (last >> 1) | (1 << 15) | 1;
  }
}

uint16_t anim_wanderingDot1_cc(uint16_t last, uint32_t ms) {
  last <<= 1;
  return (last == 0) ? 1 : last;
}

typedef uint16_t (*animFuncPtr)(uint16_t, uint32_t); 
volatile animFuncPtr animations[] = {
  anim_ring_cc,
  anim_ring_cw,
  anim_wanderingDot1_cc,
};
volatile uint16_t animStates[sizeof(animations)];

volatile byte currentAnim = 3;
volatile uint16_t brightness = 1;
volatile uint32_t animationDelay = 5; // in milliseconds
volatile uint32_t animationTick = 0;
volatile uint16_t pwm_tick = MAX_BRIGHTNESS;
volatile uint16_t currentState;
volatile uint16_t currentMask;

ISR(TIMER1_COMPA_vect) {
  if (++pwm_tick >= MAX_BRIGHTNESS) {
    pwm_tick = 0;
    if (brightness == 0) {
      digitalWrite(enablePin, HIGH); // disable it  
    } else {
      digitalWrite(enablePin, LOW); // enable it  
    }
    pulse(latchPin, HIGH); // transfer last state to outputs
    currentMask = 1 << 15; // initialize mask for next shifting cycle (pwm_ticks 1..16)

    if (currentAnim == 0) {
      animationTick = 0;
    } else {
      if (animationTick == 0) {
        byte i = currentAnim - 1;
        uint16_t oldState = animStates[i];
        currentState = animations[i](oldState, 0);
        animationTick = animationDelay;
        animStates[i] = currentState;
      } else {
        animationTick--;
      }
    }
  } else {
    if (pwm_tick == brightness) {
      digitalWrite(enablePin, HIGH); // disable it  
    }
    if (pwm_tick <= 16) {
      //Serial.println(currentMask, HEX);
      if (currentMask & currentState) {
        digitalWrite(dataPin, HIGH);
      } else {
        digitalWrite(dataPin, LOW);
      }
      pulse(clockPin, HIGH);
      currentMask >>= 1;
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
    int a, b, c;
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
        if ((a < 0) || (a > sizeof(animations))) {
          Serial.print("no such animation: ");
          Serial.println(a);
        } else {
          Serial.print("animation: ");
          if (a == 0) {
            Serial.println("none");
          } else {
            Serial.println(a);
          }
          if (a != currentAnim) {
            currentAnim = a;
          }
        }
        break;
      case CMD_DELAY:
        a = arguments[0];
        if (a < 0) {
          a = 0;
        }
        Serial.print("delay: ");
        Serial.println(a);
        animationDelay = a;
        break;

      default:
        Serial.print("unknown command ");
        Serial.println(cmd);
    }
  } else {
    serialConn = false;  
  }

  delay(100);
}


