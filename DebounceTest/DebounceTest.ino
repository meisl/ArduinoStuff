
// to disassemble:
// c:\Dokumente und Einstellungen\Administrator\Lokale Einstellungen\Temp\builde4ffd1dece802e2f6360a136b21afb9d.tmp>f:\arduino-1.6.9\hardware\tools\avr\bin\avr-objdump.exe -S DebounceTest.ino.elf > DebounceTest.disS

/*
 Debounce

 Each time the input pin goes from LOW to HIGH (e.g. because of a push-button
 press), the output pin is toggled from LOW to HIGH or HIGH to LOW.  There's
 a minimum delay between toggles to debounce the circuit (i.e. to ignore
 noise).

 The circuit:
 * LED attached from pin 13 to ground
 * pushbutton attached from pin 2 to +5V
 * 10K resistor attached from pin 2 to ground

 * Note: On most Arduino boards, there is already an LED on the board
 connected to pin 13, so you don't need any extra components for this example.


 created 21 November 2006
 by David A. Mellis
 modified 30 Aug 2011
 by Limor Fried
 modified 28 Dec 2012
 by Mike Walters

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/Debounce
 */

// set pin numbers:
#define buttonPin 2    // the number of the pushbutton pin
#define ledPin 13      // the number of the LED pin

#define buttonPort    digitalPinToPort(buttonPin)
#define buttonBitMask digitalPinToBitMask(buttonPin)



#define TIMER1_PRESCALE_STOPPED 0
#define TIMER1_PRESCALE_BY_1    (                       _BV(CS10))  //  16.000 MHz /  62.5 ns
#define TIMER1_PRESCALE_BY_8    (           _BV(CS11)            )  //   2.000 MHz / 500.0 ns
#define TIMER1_PRESCALE_BY_64   (           _BV(CS11) | _BV(CS10))  // 250.000 KHz /   4.0 µs
#define TIMER1_PRESCALE_BY_256  (_BV(CS12)                       )  //  62.500 KHz /  16.0 µs
#define TIMER1_PRESCALE_BY_1024 (_BV(CS12)            | _BV(CS10))  //  15.625 KHz /  64.0 µs



// Variables will change:
int ledState = HIGH;         // the current state of the output pin
byte initialButtonState;
byte lastButtonState;

#define backlog_size 100
uint16_t backlog[backlog_size];
volatile byte backlog_idx_in = 0;
volatile byte backlog_idx_out = 0;
volatile byte backlog_pending = 0;
volatile int backlog_missed = 0;


bool write_backlog(uint16_t t) {
  if (backlog_pending < backlog_size) {
    if (backlog_idx_in == backlog_size) { // wrap around at end
      backlog_idx_in = 0;  
    }
    backlog[backlog_idx_in++] = t;
    backlog_pending++;
    return true;
  } else { // backlog_idx_in < backlog_idx_out
    backlog_missed++;
    return false;
  }
}

bool read_backlog(uint16_t* data_ptr) {
  if (backlog_pending > 0) {
    if (backlog_idx_out == backlog_size) { // wrap around at end
      backlog_idx_out = 0;
    }
    *data_ptr = backlog[backlog_idx_out++];
    backlog_pending--;
    if (backlog_idx_out == backlog_idx_in) {
      //backlog_missed = 0;  
    }
    return true;
  }
  return false;
}
/*
long lastChanged = 0;
void onButtonChange() {
  bool currentButtonState = (*portInputRegister(buttonPort) & buttonBit) ? HIGH : LOW;
  if (currentButtonState == lastButtonState) {
    backlog_missed++;  
  }
    long now = micros();
    long data = now - lastChanged;
    data >>= 2;
    if (data > 0xFFFF) {
      data = 0;  
    }
    if (currentButtonState) {
      data |= 0x0001;
    } else {
      data &= 0xFFFE;
    }
    write_backlog(data);
    lastChanged = now;
    lastButtonState = currentButtonState;

}
*/


uint16_t ticksSinceEdge = 0;

ISR(TIMER1_COMPA_vect) {
  if (++ticksSinceEdge == 0) {
     ticksSinceEdge = 0x7FFF;
  }
  byte currentButtonState = (*portInputRegister(buttonPort) & buttonBitMask);
//  byte currentButtonState = PORTD & buttonBitMask;
  if (currentButtonState != lastButtonState) {
    uint16_t data = ticksSinceEdge << 1;
    if (currentButtonState) {
      data |= 1;  
    }
    write_backlog(data);
    ticksSinceEdge = 0;
    lastButtonState = currentButtonState;
  }
}

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  noInterrupts();
  initialButtonState = digitalRead(buttonPin);
  lastButtonState = initialButtonState ? buttonBitMask : 0;
  //attachInterrupt(digitalPinToInterrupt(buttonPin), onButtonChange, CHANGE);
  
  // Timer 1 (16 bit)
  TCCR1A =  ((1 << WGM11) & 0x00) | ((1 << WGM10) & 0x00);                          // TIMER1 CTC mode ("Clear Timer on Compare")
  TCCR1B =  ((1 << WGM13) & 0x00) | ((1 << WGM12) & 0xFF) | TIMER1_PRESCALE_BY_8;   // TIMER1 CTC mode, @2MHz
  TIMSK1 |= (1 << OCIE1A);  // enable compare interrupt for TIMER1
  OCR1A  = (10 * 2) - 1;    // compare match register: interrupt every 10 µs ~> frequency 100 kHz

  interrupts();
  
  

  // set initial LED state
  digitalWrite(ledPin, ledState);

  Serial.begin(57600);
}



void loop() {
  static bool firstTime = true;
  static int buttonState;
  if (Serial) {
    uint16_t t;
    int burstCount = 0;
    uint32_t burstTime = 0;
    if (firstTime) {
      /*
      Serial.print("button port: ");
      Serial.println(buttonPort);
      Serial.print("button bitmask: ");
      Serial.println(buttonBitMask);
      
      Serial.print("initialButtonState: ");
      Serial.println(initialButtonState);
      Serial.print("lastButtonState: ");
      Serial.println(lastButtonState);
      */
      buttonState = initialButtonState;
      firstTime = false;
    }
    while (read_backlog(&t)) {
      buttonState = (t & 1);
      t >>= 1;
      if (t > 2000) { // 20 ms
        burstCount = 0;
        burstTime = 0;
        //Serial.print("\t");  
        for (int i = 10; i > 0; i--) { Serial.println(1 - buttonState); }
        for (int i = 40; i > 0; i--) { Serial.println((1 - buttonState) * 8 - 4); }
        for (int i = 10; i > 0; i--) { Serial.println(1 - buttonState); }
      } else {
        burstTime += t;
        burstCount++;
        if (t > 500) {
          for (int i = 10; i > 0; i--) { Serial.println(1 - buttonState); }
          for (int i = 40; i > 0; i--) { Serial.println((1 - buttonState) * 8 - 4); }
          for (int i = 10; i > 0; i--) { Serial.println(1 - buttonState); }
        } else {
          for (int i = t; i > 0; i--) { Serial.println(1 - buttonState); }
        }
        //Serial.print("*\t");  
      }
      Serial.println(buttonState);
/*
      if (buttonState) {
        Serial.print("\t|__");
      } else {
        Serial.print("\t__|");
      }
      Serial.print("\t");
      Serial.print(t);
      Serial.print("0 micros (burst ");
      Serial.print(burstCount);
      Serial.print(", ");
      Serial.print(burstTime);
      Serial.print("0 micros / ");
      Serial.print(backlog_missed);
      Serial.println(" missed)");
 */
    
    }
 
  }



  /*
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        ledState = !ledState;
      }
    }
  }


  
  // set the LED:
  digitalWrite(ledPin, ledState);

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;

  */
}

