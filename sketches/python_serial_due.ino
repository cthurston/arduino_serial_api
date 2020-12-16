#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Stepper.h>

void Version() {
  Serial.println(F("V0.8"));
}

String debugMessage = "";

const char endChar = '!';
String inputString = "";
bool stringComplete = false;

Servo servos[8];
int servo_pins[] = {0, 0, 0, 0, 0, 0, 0, 0};
boolean connected = false;

const int maxDelayedExecutions = 20;
int delayTypes[maxDelayedExecutions];
int delayPins[maxDelayedExecutions];
long delayExpirations[maxDelayedExecutions];
String delayCommands[maxDelayedExecutions];

int currentStepperSpeed = 10; // less than 200
int currentStepperPosition = 0;
int stepperSteps = 200;
int stepperPin1 = 23;
int stepperPin2 = 25;
int stepperPin3 = 27;
int stepperPin4 = 29;

Stepper stepper(stepperSteps, stepperPin1, stepperPin2, stepperPin3, stepperPin4);



int Str2int (String Str_value)
{
  char buffer[10]; // max length is three units
  Str_value.toCharArray(buffer, 10);
  int int_value = atoi(buffer);
  return int_value;
}

void split(String results[], int len, String input, char spChar) {
  String temp = input;
  for (int i = 0; i < len; i++) {
    int idx = temp.indexOf(spChar);
    results[i] = temp.substring(0, idx);
    temp = temp.substring(idx + 1);
  }
}

void DigitalHandler(int mode, String data) {
  int pin = Str2int(data);
  if (mode <= 0) { //read
    Serial.println(digitalRead(pin));
  } else {
    if (pin < 0) {
      digitalWrite(-pin, LOW);
    } else {
      digitalWrite(pin, HIGH);
    }
  }
}

void AnalogHandler(int mode, String data) {
  if (mode <= 0) { // read
    int pin = Str2int(data);
    Serial.println(analogRead(pin));
  } else {
    String sdata[2];
    split(sdata, 2, data, '%');

    int pin = getAnalogPin(sdata[0]);

    int pv = Str2int(sdata[1]);
    if (pv > 4095) {
      pv = 4095;
    } else if (pv < 0) {
      pv = 0;
    }
    analogWrite(pin, pv);
  }
}

int getAnalogPin(String pinData) {
  int pin = 0;
  if (String("DAC0").equals(pinData) || String("dac0").equals(pinData)) {
    pin = DAC0;
  } else if (String("DAC1").equals(pinData) || String("dac1").equals(pinData)) {
    pin = DAC1;
  } else {
    pin = Str2int(pinData);
  }

  return pin;
}

void ConfigurePinHandler(String data) {
  int pin = Str2int(data);
  if (pin <= 0) {
    pinMode(-pin, INPUT);
  } else {
    pinMode(pin, OUTPUT);
  }
}

void shiftOutHandler(String data) {
  String sdata[4];
  split(sdata, 4, data, '%');
  int dataPin = sdata[0].toInt();
  int clockPin = sdata[1].toInt();
  String bitOrderName = sdata[2];
  byte value = (byte)(sdata[3].toInt());
  if (bitOrderName == "MSBFIRST") {
    shiftOut(dataPin, clockPin, MSBFIRST, value);
  } else {
    shiftOut(dataPin, clockPin, LSBFIRST, value);
  }
}

void shiftInHandler(String data) {
  String sdata[3];
  split(sdata, 3, data, '%');
  int dataPin = sdata[0].toInt();
  int clockPin = sdata[1].toInt();
  String bitOrderName = sdata[2];
  int incoming;
  if (bitOrderName == "MSBFIRST") {
    incoming = (int)shiftIn(dataPin, clockPin, MSBFIRST);
  } else {
    incoming = (int)shiftIn(dataPin, clockPin, LSBFIRST);
  }
  Serial.println(incoming);
}

void pulseInHandler(String data) {
  int pin = Str2int(data);
  long duration;
  if (pin <= 0) {
    pinMode(-pin, INPUT);
    duration = pulseIn(-pin, LOW);
  } else {
    pinMode(pin, INPUT);
    duration = pulseIn(pin, HIGH);
  }
  Serial.println(duration);
}

void pulseInSHandler(String data) {
  int pin = Str2int(data);
  long duration;
  if (pin <= 0) {
    pinMode(-pin, OUTPUT);
    digitalWrite(-pin, HIGH);
    delayMicroseconds(2);
    digitalWrite(-pin, LOW);
    delayMicroseconds(5);
    digitalWrite(-pin, HIGH);
    pinMode(-pin, INPUT);
    duration = pulseIn(-pin, LOW);
  } else {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);
    duration = pulseIn(pin, HIGH);
  }
  Serial.println(duration);
}

void SV_add(String data) {
  String sdata[3];
  split(sdata, 3, data, '%');
  int pin = Str2int(sdata[0]);
  int min = Str2int(sdata[1]);
  int max = Str2int(sdata[2]);
  int pos = -1;
  for (int i = 0; i < 8; i++) {
    if (servo_pins[i] == pin) { //reset in place
      servos[pos].detach();
      servos[pos].attach(pin, min, max);
      servo_pins[pos] = pin;
      Serial.println(pos);
      return;
    }
  }
  for (int i = 0; i < 8; i++) {
    if (servo_pins[i] == 0) {
      pos = i;  // find spot in servo array
      break;
    }
  }
  if (pos == -1) {;} //no array position available!
  else {
    servos[pos].attach(pin, min, max);
    servo_pins[pos] = pin;
    Serial.println(pos);
  }
}

void SV_remove(String data) {
  int pos = Str2int(data);
  servos[pos].detach();
  servo_pins[pos] = 0;
}

void SV_read(String data) {
  int pos = Str2int(data);
  int angle;
  angle = servos[pos].read();
  Serial.println(angle);
}

void SV_write(String data) {
  String sdata[2];
  split(sdata, 2, data, '%');
  int pos = Str2int(sdata[0]);
  int angle = Str2int(sdata[1]);
  servos[pos].write(angle);
}

void SV_write_ms(String data) {
  String sdata[2];
  split(sdata, 2, data, '%');
  int pos = Str2int(sdata[0]);
  int uS = Str2int(sdata[1]);
  servos[pos].writeMicroseconds(uS);
}

void StepperInitialize(String data) {
  String sdata[2];
  split(sdata, 2, data, '%');
  currentStepperSpeed = Str2int(sdata[0]);
  int pos = Str2int(sdata[1]);

  // If they provide a negative position then do not reset position on init
  if (pos >= 0) currentStepperPosition = pos;
  stepper.setSpeed(currentStepperSpeed);
}

void StepperStep(String data) {
  int steps = Str2int(data);
  stepper.step(steps);
  updateStepperPosition(steps);
}

void StepperStepGoto(String data) {
  int pos = Str2int(data);

  int forward = pos - currentStepperPosition;
  int backward = pos - (currentStepperPosition - stepperSteps);

  int steps = forward;
  if (abs(forward) > abs(backward)) steps = backward;

  stepper.step(steps);
  updateStepperPosition(steps);
}

void updateStepperPosition(int steps) {
  currentStepperPosition = (currentStepperPosition + steps) % stepperSteps;
  if (currentStepperPosition < 0) currentStepperPosition += stepperSteps;
}

void StepperPos() {
  Serial.println(currentStepperPosition);
}
void StepperSpeed(int read, String data) {
  if (read == 1) Serial.println(currentStepperSpeed);
  if (read == 0) {
    int speed = Str2int(data);
    stepper.setSpeed(speed);
    currentStepperSpeed = speed;
  }
}

void SPIInitialization(String data) {
  SPI.begin();
  SPI.setDataMode(SPI_MODE2);
  SPI.setClockDivider(SPI_CLOCK_DIV4); // configuration of clock at 4MHz

  String sdata[1];
  split(sdata, 1, data, '%');
  int syncPin = Str2int(sdata[0]);

  pinMode(syncPin, OUTPUT);

  digitalWrite(syncPin, LOW);
  delay(1);
  digitalWrite(syncPin, HIGH);
}

void SPIHandler(int mode, String data) {
  String sdata[3];
  split(sdata, 3, data, '%');
  int pin = Str2int(sdata[0]);
  int b1 = Str2int(sdata[1]);
  int b2 = Str2int(sdata[2]);

  digitalWrite(pin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(pin, HIGH);
}

void DelayedHandler(int type, String data) {
  // Expect pin#delayMS#pin%value%...
  String sdata[3];
  split(sdata, 3, data, '#');

  // Analog writes could be on the dac
  int pin = 0;
  if (type == 1) {
    pin = getAnalogPin(sdata[0]);
  } else {
    pin = sdata[0].toInt();
  }

  int delay = sdata[1].toInt();
  String d = sdata[2];

  CancelDelayed(pin);

  long expires = delay + millis();

  int found = 0;
  for (int i = 0; i < maxDelayedExecutions; i += 1) {
    if (found == 0 && delayExpirations[i] == 0) {
      delayTypes[i] = type;
      delayPins[i] = pin;
      delayExpirations[i] = expires;
      delayCommands[i] = d;
      found = 1;
      break;
    }
  }
}

void ExecuteDelayed() {
  int now = millis();
  for (int i = 0; i < maxDelayedExecutions; i += 1) {
    // Is this time to execute
    if (delayExpirations[i] != 0 && delayExpirations[i] <= now) {
      String data = delayCommands[i];
      int type = delayTypes[i];

      if (type == 0) {
        AnalogHandler(1, data);
      } else if (type == 1) {
        DigitalHandler(1, data);
      } else if (type == 2) {
        SPIHandler(1, data);
      }

      // Remove this execution from the list
      CancelDelayed(delayPins[i]);
    }
  }
}

void CancelDelayed(int pin) {
  for (int i = 0; i < maxDelayedExecutions; i += 1) {
    if (delayPins[i] == pin) {
      delayExpirations[i] = 0;
    }
  }
}

void DebugHandler(String data) {
  Serial.println(debugMessage);
  debugMessage = "";
}

void SerialParser(String read_) {
  int idx1 = read_.indexOf('%');
  int idx2 = read_.indexOf('$');
  // separate command from associated data
  String cmd = read_.substring(1, idx1);
  String data = read_.substring(idx1 + 1, idx2);

  inputString = "";
  stringComplete = false;

  // determine command sent
  if (cmd == "dw") {
    DigitalHandler(1, data);
  }
  else if (cmd == "dr") {
    DigitalHandler(0, data);
  }
  else if (cmd == "aw") {
    AnalogHandler(1, data);
  }
  else if (cmd == "ar") {
    AnalogHandler(0, data);
  }
  else if (cmd == "pm") {
    ConfigurePinHandler(data);
  }
  else if (cmd == "ps") {
    pulseInSHandler(data);
  }
  else if (cmd == "pi") {
    pulseInHandler(data);
  }
  else if (cmd == "sva") {
    SV_add(data);
  }
  else if (cmd == "svr") {
    SV_read(data);
  }
  else if (cmd == "svw") {
    SV_write(data);
  }
  else if (cmd == "svwm") {
    SV_write_ms(data);
  }
  else if (cmd == "svd") {
    SV_remove(data);
  }
  else if (cmd == "version") {
    Version();
  }
  else if (cmd == "so") {
    shiftOutHandler(data);
  }
  else if (cmd == "si") {
    shiftInHandler(data);
  }
  else if (cmd == "spii") {
    SPIInitialization(data);
  }
  else if (cmd == "spiw") {
    SPIHandler(1, data);
  }
  else if (cmd == "daw") {
    // Delayed analog write
    DelayedHandler(0, data);
  }
  else if (cmd == "ddw") {
    // Delayed digital write
    DelayedHandler(1, data);
  }
  else if (cmd == "dspiw") {
    DelayedHandler(2, data);
  }
  else if (cmd == "stpi") {
    StepperInitialize(data);
  }
  else if (cmd == "stps") {
    StepperStep(data);
  }
  else if (cmd == "stpgo") {
    StepperStepGoto(data);
  }
  else if (cmd == "stpsp") {
    StepperSpeed(0, data);
  }
  else if (cmd == "rstpsp") {
    StepperSpeed(1, "");
  }
  else if (cmd == "rstps") {
    StepperPos();
  }
  else if (cmd == "debug") {
    DebugHandler(data);
  }
}

void setup()  {
  analogWriteResolution(12); // 12 bit for DACs
  Serial.begin(115200);
  Serial.println("connected");

  // Stepper setup
  pinMode(stepperPin1, OUTPUT);
  pinMode(stepperPin2, OUTPUT);
  pinMode(stepperPin3, OUTPUT);
  pinMode(stepperPin4, OUTPUT);
  digitalWrite(stepperPin1, LOW);
  digitalWrite(stepperPin2, LOW);
  digitalWrite(stepperPin3, LOW);
  digitalWrite(stepperPin4, LOW);
}

void loop() {
  ExecuteDelayed();
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is our end char, set a flag so the main loop can
    // do something about it:
    if (inChar == endChar) {
      stringComplete = true;
      SerialParser(inputString);
    }
  }
}