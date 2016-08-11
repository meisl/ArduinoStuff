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

// constants won't change. They're used here to
// set pin numbers:
const int buttonPin = 2;    // the number of the pushbutton pin
const int ledPin = 13;      // the number of the LED pin

// Variables will change:
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
int initialButtonState;

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

long t_edge = 0;

#define backlog_size 100
long backlog[backlog_size];
volatile int backlog_idx_in = 0;
volatile int backlog_idx_out = 0;
volatile int backlog_pending = 0;
volatile long backlog_missed = 0;


bool write_backlog(long t) {
  int idx_delta = backlog_idx_in - backlog_idx_out;
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

long read_backlog() {
  long result = -1;
  if (backlog_pending > 0) {
    if (backlog_idx_out == backlog_size) { // wrap around at end
      backlog_idx_out = 0;
    }
    result = backlog[backlog_idx_out++];
    backlog_pending--;
    if (backlog_idx_out == backlog_idx_in) {
      backlog_missed = 0;  
    }
    return result;
  }
  return result;
}

long lastChanged = 0;
void onButtonChange() {
  long now = micros();
  write_backlog(now - lastChanged);
  lastChanged = now;
}

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  noInterrupts();
  initialButtonState = digitalRead(buttonPin);
  attachInterrupt(digitalPinToInterrupt(buttonPin), onButtonChange, CHANGE);
  interrupts();

  // set initial LED state
  digitalWrite(ledPin, ledState);

  Serial.begin(57600);
}


void loop() {
  static bool firstTime = true;
  static int buttonState;
  if (Serial) {
    long t;
    int burstCount = 0;
    float burstTime = 0;
    if (firstTime) {
      Serial.print("initial button state: ");
      Serial.println(initialButtonState);
      buttonState = initialButtonState;
      firstTime = false;
    }
    while ( (t = read_backlog()) >= 0) {
      buttonState = !buttonState;
      float millis = float(t) / 1000.0;
      if (millis > 20) {
        burstCount = 0;
        burstTime = 0;
        Serial.println("--------------");  
      } else {
        burstTime += millis;
        burstCount++;
      }
      Serial.print(buttonState);
      Serial.print("\t");
      Serial.print(millis);
      Serial.print(" ms (burst ");
      Serial.print(burstCount);
      Serial.print(", ");
      Serial.print(burstTime);
      Serial.print(" ms / ");
      Serial.print(backlog_missed);
      Serial.println(" missed)");
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

