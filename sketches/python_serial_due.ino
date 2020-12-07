#include <Wire.h>
#include <Servo.h>
#include <SPI.h>

void Version() {
  Serial.println(F("V0.7"));
}

Servo servos[8];
int servo_pins[] = {0, 0, 0, 0, 0, 0, 0, 0};
int spiSyncPin = 10;
boolean connected = false;

int Str2int (String Str_value)
{
  char buffer[10]; //max length is three units
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
  if (mode <= 0) { // read
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
    int pin = 0;
    if (String("DAC0").equals(sdata[0]) || String("dac0").equals(sdata[0])) {
      pin = DAC0;
    } else if (String("DAC1").equals(sdata[0]) || String("dac1").equals(sdata[0])) {
      pin = DAC1;
    } else {
      pin = Str2int(sdata[0]);
    }
    
    int pv = Str2int(sdata[1]);
    if (pv > 4095) {
      pv = 4095;
    } else if (pv < 0) {
      pv = 0;
    }
    analogWrite(pin, pv);
  }
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

void SPIInitialization(String data) {
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV4); // configuration of clock at 4MHz

  String sdata[1];
  split(sdata, 1, data, '%');
  int syncPin = Str2int(sdata[0]);

  spiSyncPin = syncPin;
  pinMode(spiSyncPin, OUTPUT);

  digitalWrite(spiSyncPin, LOW);
  delay(1);
  digitalWrite(spiSyncPin, HIGH);
}

void SPIHandler(int mode, String data) {
  String sdata[2];
  split(sdata, 2, data, '%');
  int b1 = Str2int(sdata[0]);
  int b2 = Str2int(sdata[1]);

  digitalWrite(spiSyncPin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(spiSyncPin, HIGH);
}

void SerialParser(void) {
  char readChar[64];
  Serial.readBytesUntil(33, readChar, 64);
  String read_ = String(readChar);
  //Serial.println(readChar);
  int idx1 = read_.indexOf('%');
  int idx2 = read_.indexOf('$');
  // separate command from associated data
  String cmd = read_.substring(1, idx1);
  String data = read_.substring(idx1 + 1, idx2);

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
}

void setup()  {
  analogWriteResolution(12); // 12 bit for DACs
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("connected");
}

void loop() {
  SerialParser();
}