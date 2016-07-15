/*
  Button

 Turns on and off a light emitting diode(LED) connected to digital
 pin 13, when pressing a pushbutton attached to pin 2.


 The circuit:
 * LED attached from pin 13 to ground
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground

 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.


 created 2005
 by DojoDave <http://www.0j0.org>
 modified 30 Aug 2011
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/Button
 */

// constants won't change. They're used here to
// set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState_before;         // variables for reading the pushbutton status
int buttonState_now;
int buttonState_toToggle;
int ledState = LOW;         // initial LED state


int toggle() {
  ledState = !ledState;
  digitalWrite(ledPin, ledState); 
  return ledState;
}
  
int blink(int times, long delay_ms) {
  for (; times > 0; times--) {
    ledState = toggle();
    delay(delay_ms);  
    ledState = toggle();  
    delay(delay_ms);  
  }
  return ledState;
}


void setup() {
  pinMode(ledPin, OUTPUT);    // initialize the LED pin as an output
  pinMode(buttonPin, INPUT);  // initialize the pushbutton pin as an input
  digitalWrite(ledPin, ledState);  // set initial LED state
  Serial.begin(57600);
  while (!Serial);
  Serial.print("ToggleLED ready: LED will toggle on ");
  delay(1);
  Serial.print(buttonState_toToggle == HIGH ? "rising" : "falling");
  delay(1);
  Serial.println(" edge.");
  delay(1);
  
  ledState = blink(12, 50);
  buttonState_toToggle = digitalRead(buttonPin);
  buttonState_before = buttonState_toToggle;
}

void loop() {
  buttonState_now = digitalRead(buttonPin); // read the state of the pushbutton value
  if (buttonState_now != buttonState_before) {
    if (buttonState_now == buttonState_toToggle) {
      ledState = toggle();
    }
    buttonState_before = buttonState_now;
  }
}


