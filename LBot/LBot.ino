

#include <Wire.h>
#include <VL53L0X.h>  // laser range finder
#include <Servo.h>

VL53L0X rangeFinder;
Servo servo;
const byte pinServo = 7;

// for checking serial input
bool newInput = false;
byte input;

// for sending output
unsigned long lastSent = 0;
unsigned long tm;

// distance readings
// there will be 17 (from 10 to 170 degrees)
const byte numberOfReadings = 17;
int rangeAngles[numberOfReadings];
int rangeReadings[numberOfReadings];


void setup() {
  Serial.begin(115200); // to connect to computer
  Serial3.begin(115200); // to connect to BT module
  setupServo();
  Wire.begin();
  setupRangeFinder();
  Serial.println(F("Setup complete"));
} // END LOOP


void loop() {

  checkForInput();
  showInput();
  takeRangeReadings();

  tm = millis();
  if (tm - lastSent > 1000) {
    sendData();
    lastSent += 1000;
  }
} // END LOOP



void takeRangeReadings() {
  for (byte i = 0; i < numberOfReadings; i++) {
    servo.write(rangeAngles[i]);
    delay(50);
    rangeReadings[i] = rangeFinder.readRangeSingleMillimeters();
  }
}


// only expecting single character input (at the moment)
void checkForInput() {
  if (Serial3.available()) {
    input = Serial3.read();
    newInput = true;
  }
}

void showInput() {
  if (newInput) {
    Serial.print("Data received: ");
    Serial.print(input);
    Serial.print('\n');
    newInput = false;
  }
}

void sendData() {
  static int inc = 0;
  // to serial monitor
  Serial.print(tm); Serial.print('\t');
  Serial.print(inc); Serial.print('\t');
  for (int i = 0; i < numberOfReadings; i++) {
    Serial.print(rangeReadings[i]);
    if (i != (numberOfReadings - 1)) Serial.print('\t');
  }
  Serial.print('\n');
  // to BT module
  Serial3.print(tm); Serial3.print('\t');
  Serial3.print(inc); Serial3.print('\t');
  for (int i = 0; i < numberOfReadings; i++) {
    Serial3.print(rangeReadings[i]);
    if (i != (numberOfReadings - 1)) Serial3.print('\t');
  }
  Serial3.print('\n');

  inc += 1;
}

void setupServo() {
  servo.write(10);
  servo.attach(pinServo);
  for (byte i = 0; i < numberOfReadings; i++) {
    rangeAngles[i] = 10 + 10 * i;
    //    Serial.println(rangeAngles[i]);
  }
}

void setupRangeFinder() {
  rangeFinder.init();
  rangeFinder.setTimeout(500);
  // lower the return signal rate limit (default is 0.25 MCPS)
  rangeFinder.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  rangeFinder.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  rangeFinder.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

