
// The pins used to communicate with the shift registers (74HC595)
#define enablePin 12 // active-low; aka OE
#define clearPin  11 // active-low; aka MR
#define latchPin  10 // active-high; aka STCP/ST_CP
#define clockPin   9 // active-high; aka SHCP/SH_CP
#define dataPin    8 // active-high; aka DS

// You can connect LEDs (with resistor) between the above 
// pins and GND to see what's going on. But in order to be
// able to see anything useful, things need to be slowed down:
#define visible_signals
#define signal_duration_ms 50


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
  
  Serial.begin(9600);
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


bool serialConn = false;
uint16_t data = 0;

void loop() {
  if (~data == 0) {
    data = 0;
    reset595();
  }
  data <<= 1;
  data |= 1;

  //shiftOut(dataPin, clockPin, MSBFIRST, highByte(data));  
  //shiftOut(dataPin, clockPin, MSBFIRST, lowByte(data));


  if (data & 1) {
    digitalWrite(dataPin, HIGH);  
  }
  pulse(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
 
  pulse(latchPin, HIGH);


  if (Serial) {
    if (!serialConn) { // greet if reconnected
      Serial.println("Hi there!");
      serialConn = true;
    }
    Serial.println(data);
  } else {
    serialConn = false;  
  }
  
  delay(100);
}


