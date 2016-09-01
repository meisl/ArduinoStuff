
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
      arguments[0] = Serial.parseInt();
      break;
  }

  return c;
}

void setBrightness(byte b) {
  if (b == 0) {
    digitalWrite(enablePin, HIGH);
  } else {
    digitalWrite(enablePin, LOW);
  }
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
    byte cmd = parseCommand();
    int a, b, c;
    switch (cmd) {
      case CMD_NONE:
      case CMD_EMPTY:
        break;
      case CMD_BRIGHTNESS:
        Serial.print("brightness: ");
        a = constrain(arguments[0], 0, 15);
        Serial.println(a);
        setBrightness(a);
        break;
      default:
        Serial.print("unknown command ");
        Serial.println(cmd);
    }
    
    Serial.println(data, HEX);
    
  } else {
    serialConn = false;  
  }
  
  delay(100);
}


