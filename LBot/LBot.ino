

#include <Wire.h>
#include <VL53L0X.h>  // laser range finder
#include <Servo.h>

VL53L0X rangeFinder;
Servo servo;
const byte pinServo = 7;



// for incoming comms
bool newDataReceived = false;
byte newCommandType;  // what type of data has been sent
//  expecting 1, 2, 3
const char charArrayTerminator = '\0';
const byte rxBufferSize = 15; // longest expected atm is 11 (+string termination char)
static char rxBuffer[rxBufferSize];

// for outgoing comms
const byte MOTION_PACKET_ID = 1;
const byte SENSOR_PACKET_ID = 2;
const byte CONTROL_PACKET_ID = 3;

// for movement
float nextTurnAngle, nextMoveForward;
float defaultTurnAngle = 0.0;   // in degrees
float defaultMoveForward = 200.0; // in millimeters
bool moveRequestReceived = false; // set to true after receiving a command to move // set to false after movement is done

// for recording movement
const float wheelBase = 20; // in millimeters // placeholder
const float wheelCircumference = 80; // in millimeters // placeholder
float angleTurned;
float distanceMoved;
const int encoderTicksPerRevolution = 100;  // placeholder

// for motors
const byte pinLeftMotorDirection = 4;
const byte pinRightMotorDirection = 5;
const byte pinLeftMotorPWM = 10;
const byte pinRightMotorPWM = 11;
const byte pinLeftEncoderA = 2;
const byte pinLeftEncoderB = 8;
const byte pinRightEncoderA = 3;
const byte pinRightEncoderB = 9;
volatile int leftEncoderCounter = 0;
volatile int rightEncoderCounter = 0;
byte baseSpeed = 150;

// for sending output
unsigned long lastSent = 0;
unsigned long tm;

// distance readings
// there will be 17 (from 10 to 170 degrees)
const byte numberOfReadings = 17;
int rangeAngles[numberOfReadings];
int rangeReadings[numberOfReadings];




void setup() {
  randomSeed(analogRead(0));

  Serial.begin(115200); // to connect to computer
  Serial3.begin(115200); // to connect to BT module
  setupServo();
  Wire.begin();
  setupRangeFinder();
  setupMotorsAndEncoders();
  Serial.println(F("Setup complete"));
} // END LOOP


void loop() {

//  readSerialToBuffer();
//  if (newDataReceived) parseData();
//
//  if (moveRequestReceived) {
//    Serial.print(newCommandType); Serial.print('\t');
//    Serial.print(nextTurnAngle); Serial.print('\t');
//    Serial.print(nextMoveForward); Serial.print('\n');
//    makeMovement();
//    sendMotionData();
//    takeRangeReadings();
//    sendSensorData();
//    moveRequestReceived = false;
//  }

  // testing by moving manually
  Serial.print(leftEncoderCounter);Serial.print('\t');
  Serial.print(rightEncoderCounter);Serial.print('\n');
  delay(1000);


} // END LOOP



void takeRangeReadings() {
  for (byte i = 0; i < numberOfReadings; i++) {
    servo.write(rangeAngles[i]);
    delay(50);
    rangeReadings[i] = rangeFinder.readRangeSingleMillimeters();
    if (rangeReadings[i] > 2000) rangeReadings[i] = 0; // above ~2000 the sensor will return ~8000
  }
}

//////////////////////////////////////////////////////
// FOR RECEIVING /////////////////////////////////////
//////////////////////////////////////////////////////

void readSerialToBuffer() {
  const char endMarker = '\n';
  static byte idx = 0;
  static bool receivingInProgress = false;  // indicate if we are midway through receiving something
  char inputChar;
  // read anything that's available from the BT module
  if (Serial3.available() > 0) {
    inputChar = Serial3.read();
    //    Serial.println(inputChar);
  }
  else return;  // ** NOTE RETURN **
  // if this is the beginning of a new transmission
  if (!receivingInProgress) {
    receivingInProgress = true; // indicate that we are now part way through a transmission
  }
  if (inputChar != endMarker) {
    rxBuffer[idx] = inputChar;
    idx++;
    if (idx >= rxBufferSize) {  // just in case we have overrun the buffer
      idx = rxBufferSize - 1;
    }
  }
  else {
    rxBuffer[idx] = charArrayTerminator; // terminate the string
    idx = 0;
    newDataReceived = true;
    receivingInProgress = false;
  }
}


// general function that will perform basic sense check on the data
// determine what command type it is, and send to a command specific parser
// should this also split into parts? prob yes
void parseData() {
  // ignore command type for a moment
  const byte elementBufferSize = 6;
  const char delimiter = '\t';
  char elementBuffer[elementBufferSize];
  byte elementIdx = 0;
  //  bool elementComplete = false;
  int output;
  bool firstElement = true;
  for (byte rxIdx = 0; rxIdx < rxBufferSize; rxIdx++) {
    if (rxBuffer[rxIdx] == delimiter || rxBuffer[rxIdx] == charArrayTerminator) {
      // element complete, add terminator and convert to number
      elementBuffer[elementIdx] = charArrayTerminator;
      //if this was the first element then save as newCommandType
      if (firstElement) {
        newCommandType = (byte)atoi(elementBuffer);
        firstElement = false;
      }
      // if not the first element, then send to the command specific parser
      else {
        switch (newCommandType) {
          case 1: // standard input
            processCommandType1(elementBuffer);
            break;
          case 2: // standard input
            processCommandType2(elementBuffer);
            break;
          case 3: // standard input
            processCommandType3(elementBuffer);
            break;
        }
      }
      // if the end of data has been reached then parsing is complete
      if (rxBuffer[rxIdx] == charArrayTerminator) {
        newDataReceived = false;
        return;
      }
      // if not then reset the element buffer index for the next element
      elementIdx = 0;
    }
    else {  // valid data, extract to element buffer
      elementBuffer[elementIdx] = rxBuffer[rxIdx];
      elementIdx++;
    }
  }
}


// THERE IS NO ERROR CHECKING I.E. DO I GET THE RIGHT NUMBER OF NUMBERS?
void processCommandType1 (char* element) {
  // standard command to move a set distance and read the sensors
  // this will only be called once
  // at the moment, not even bothering to check the value of the command
  //  Serial.println("xxx");
  nextTurnAngle = defaultTurnAngle;
  nextMoveForward = defaultMoveForward;
  moveRequestReceived = true;
}

// THERE IS NO ERROR CHECKING I.E. DO I GET THE RIGHT NUMBER OF NUMBERS?
void processCommandType2 (char* element) {
  // this will be called multiple times
  // required turn angle, required distance forward
  static byte variableCounter = 0;
  const byte variableMax = 2; // expecting a maximum of 2 values
  //  Serial.println("I am here");
  switch (variableCounter) {
    case 0: // first variable, turn angle
      nextTurnAngle = atof(element);
      variableCounter++;
      //      Serial.println(nextTurnAngle);
      break;
    case 1: // first variable, move distance
      nextMoveForward = atof(element);
      //      Serial.println(nextMoveForward);
      variableCounter = 0;
      moveRequestReceived = true; // that's everything done, so flag that we have a new move request ready to act upon
      break;
  }
}

// THERE IS NO ERROR CHECKING I.E. DO I GET THE RIGHT NUMBER OF NUMBERS?
void processCommandType3 (char* elementBuffer) {
  // not yet defined
}



//////////////////////////////////////////////////////
// FOR SENDING ///////////////////////////////////////
//////////////////////////////////////////////////////

void sendMotionData() {
  static int inc = 0;
  tm = millis();
  // to BT module
  Serial3.print('1'); Serial3.print('\t');
  Serial3.print(tm); Serial3.print('\t');
  Serial3.print(inc); Serial3.print('\t');
  Serial3.print(angleTurned, 5); Serial3.print('\t');
  Serial3.print(distanceMoved);
  Serial3.print('\n');
  inc += 1;
  //  Serial.println(angleTurned,5);
}


void sendSensorData() {
  static int inc = 0;
  tm = millis();

  // to serial monitor
  //  Serial.print(tm); Serial.print('\t');
  //  Serial.print(inc); Serial.print('\t');
  //  for (int i = 0; i < numberOfReadings; i++) {
  //    Serial.print(rangeReadings[i]);
  //    if (i != (numberOfReadings - 1)) Serial.print('\t');
  //  }
  //  Serial.print('\n');

  // to BT module
  Serial3.print('2'); Serial3.print('\t');
  Serial3.print(tm); Serial3.print('\t');
  Serial3.print(inc); Serial3.print('\t');
  for (int i = 0; i < numberOfReadings; i++) {
    Serial3.print(rangeReadings[i]);
    if (i != (numberOfReadings - 1)) Serial3.print('\t');
  }
  Serial3.print('\n');

  inc += 1;
}

//////////////////////////////////////////////////////
// SERVO /////////////////////////////////////////////
//////////////////////////////////////////////////////

void setupServo() {
  servo.write(10);
  servo.attach(pinServo);
  for (byte i = 0; i < numberOfReadings; i++) {
    rangeAngles[i] = 10 + 10 * i;
    //    Serial.println(rangeAngles[i]);
  }
}

//////////////////////////////////////////////////////
// RANGE FINDER //////////////////////////////////////
//////////////////////////////////////////////////////

void setupRangeFinder() {
  rangeFinder.init();
  rangeFinder.setTimeout(500);
  // lower the return signal rate limit (default is 0.25 MCPS)
  rangeFinder.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  rangeFinder.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  rangeFinder.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

//////////////////////////////////////////////////////
// FOR MOVEMENT //////////////////////////////////////
//////////////////////////////////////////////////////

void setupMotorsAndEncoders() {
  pinMode(pinLeftMotorDirection, OUTPUT);
  pinMode(pinLeftMotorPWM, OUTPUT);
  pinMode(pinLeftMotorDirection, OUTPUT);
  pinMode(pinLeftMotorPWM, OUTPUT);
  digitalWrite(pinRightMotorDirection, LOW);
  digitalWrite(pinRightMotorPWM, LOW);
  digitalWrite(pinRightMotorDirection, LOW);
  digitalWrite(pinRightMotorPWM, LOW);
  pinMode(pinLeftEncoderA, INPUT);
  pinMode(pinLeftEncoderB, INPUT);
  pinMode(pinRightEncoderA, INPUT);
  pinMode(pinRightEncoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinLeftEncoderA), countLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(pinRightEncoderA), countRightEncoder, RISING);
}

void countLeftEncoder() {
//  leftEncoderCounter++;
  byte dir = bitRead(PINH, 5);   // pin 8 is port H.5 on the Mega
  if (dir == 1) {
    leftEncoderCounter++;
  }
  else {
    leftEncoderCounter--;
  }
}

void countRightEncoder() {
//  rightEncoderCounter++;
  byte dir = bitRead(PINH, 6);   // pin 9 is port H.6 on the Mega
  if (dir == 1) {
    rightEncoderCounter++;
  }
  else {
    rightEncoderCounter--;
  }
}

void makeMovement() {
  // placeholder
  //  angleTurned = (float)(random(100)-50)/50;
  //  distanceMoved = (float)random(50);
  angleTurned = 1.5708;
  distanceMoved = 200;
}

void makeMovementNew() {
//  turn();
//  calculateTurn();
  forward();
  // now check progress and stop when done
  // do I want to check all the time or periodically?
  delay(2000);
  stopMove();
//  calculateDistance();
}

void turn(){
  
}

void forward(){
  digitalWrite(pinRightMotorDirection, HIGH);
  digitalWrite(pinRightMotorPWM, baseSpeed);
  digitalWrite(pinRightMotorDirection, HIGH);
  digitalWrite(pinRightMotorPWM, baseSpeed);
}

void stopMove(){
  digitalWrite(pinRightMotorPWM, 0);
  digitalWrite(pinRightMotorPWM, 0);
}

void calculateTurn() {
  angleTurned = 1.5708;
  distanceMoved = 200;
  
}

void calculateDistance() {
  angleTurned = 1.5708;
  distanceMoved = 200;
  
}

