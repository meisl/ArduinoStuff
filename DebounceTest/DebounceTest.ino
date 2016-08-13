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

#define buttonPort digitalPinToPort(buttonPin)
#define buttonBit  digitalPinToBitMask(buttonPin)

// Variables will change:
int ledState = HIGH;         // the current state of the output pin
bool buttonState;            // the current reading from the input pin
bool lastButtonState = LOW;  // the previous reading from the input pin
bool initialButtonState;

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

long t_edge = 0;

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

long lastChanged = 0;
void onButtonChange() {
  bool currentButtonState = (*portInputRegister(buttonPort) & buttonBit) ? HIGH : LOW;
  if (currentButtonState == lastButtonState) {
    backlog_missed ++;  
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

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  noInterrupts();
  initialButtonState = digitalRead(buttonPin);
  lastButtonState = initialButtonState;
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
    uint16_t t;
    int burstCount = 0;
    float burstTime = 0;
    if (firstTime) {
      Serial.print("button port: ");
      Serial.println(buttonPort);
      Serial.print("button bitmask: ");
      Serial.println(buttonBit);
      
      Serial.print("initial button state: ");
      Serial.println(initialButtonState);
      buttonState = initialButtonState;
      firstTime = false;
    }
    while (read_backlog(&t)) {
      buttonState = (t & 1) ? HIGH : LOW;   //  !buttonState;
      float millis = 99999.99;
      if (t > 1) {
        millis = float(t) * 0.004;
      }
      if (millis > 20) {
        burstCount = 0;
        burstTime = 0;
        Serial.print("\t");  
      } else {
        burstTime += millis;
        burstCount++;
        Serial.print("*\t");  
      }
      Serial.print(buttonState);
      if (buttonState == HIGH) {
        Serial.print("\t|__");
      } else {
        Serial.print("\t__|");
      }
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

