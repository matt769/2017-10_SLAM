#include <Wire.h> // Standard I2C communication library
#include <VL53L0X.h>  // laser range finder // https://github.com/pololu/vl53l0x-arduino
#include <Servo.h>    // Standard Servo library
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

VL53L0X rangeFinder;
Servo servo;
const byte pinServo = 22;

// for incoming comms
bool newDataReceived = false;
byte newCommandType;  // what type of data has been sent  //  expecting 1, 2, 3
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
const float wheelBase = 109.0; // in millimeters
const float wheelBaseInverse = 1.0 / wheelBase;
const float wheelRadius = 30.0; // in millimeters
const float wheelCircumference = wheelRadius * 2 * PI; // in millimeters
float angleTurned;
float distanceMoved;
const int encoderTicksPerRevolution = 7 * 100;  // ticks per motor shaft turn * gear ratio
const float distancePerTick = wheelCircumference / (float)encoderTicksPerRevolution;
long distanceTargetTicks = 0;  // after receiving a distance to travel, convert to encoder ticks for quicker calculation
long turnTargetTicks = 0;
float turningCircleCircumference = wheelBase * PI;
bool inMotionForward = false;
bool inMotionTurning = false;
unsigned long speedCalcInterval;
float leftSpeedMeasured;
float rightSpeedMeasured;

// new
float v;  // velocity
float w;  // angular velocity
float theta = 0;  // to capture change in local heading
float x = 0;  // to capture change in local x direction
float y = 0;  // to capture change in local y direction
float Dl, Dr, Dc, dTheta;



// for motors
// in current wiring, left motor is M2 on the driver
const byte pinLeftMotorDirection = 10;
const byte pinRightMotorDirection = 11;
const byte pinLeftMotorPWM = 5;
const byte pinRightMotorPWM = 6;
const byte pinLeftEncoderA = 2;
const byte pinLeftEncoderB = 31;  //  31 is Port C bit 6
const byte pinRightEncoderA = 3;
const byte pinRightEncoderB = 38;  //  38 is Port D bit 7
volatile long leftEncoderCounter = 0;
volatile long rightEncoderCounter = 0;
long leftEncoderCounterCopy, rightEncoderCounterCopy;
byte baseSpeedPWM = 150;
byte leftPWM; // final PWM value
byte rightPWM;
byte leftPWMBase;
byte rightPWMBase;
float baseTurnSpeed = 1.0;
float baseForwardSpeed = 1.0;
const byte MOTOR_FORWARD = 1;
const byte MOTOR_BACKWARD = 2;
volatile int counter = 0;
unsigned long motorTimeout = 5000;


float leftSpeedSetpoint, leftSpeedAbsolute, leftPWMOutput, rightSpeedSetpoint, rightSpeedAbsolute, rightPWMOutput;
float Kp = 500, Ki = 0, Kd = 0;
PID leftSpeedPID(&leftSpeedAbsolute, &leftPWMOutput, &leftSpeedSetpoint, Kp, Ki, Kd, DIRECT);
PID rightSpeedPID(&rightSpeedAbsolute, &rightPWMOutput, &rightSpeedSetpoint, Kp, Ki, Kd, DIRECT);
unsigned long speedCalcLast;
long leftTicksLast;
long rightTicksLast;

// for sending output
unsigned long lastSent = 0;
unsigned long tm;

// distance readings
const byte minAngle = 10;
const byte maxAngle = 170;
const byte angleIncrement = 1;
const byte numberOfReadings = ((maxAngle - minAngle) / angleIncrement) + 1;
int rangeAngles[numberOfReadings];
int rangeReadings[numberOfReadings];
const byte servoDelay = 5 * angleIncrement; // allowing 5ms per degree


void setup() {
  randomSeed(analogRead(0));

  Serial.begin(115200); // to connect to computer
  Serial3.begin(115200); // to connect to BT module
  setupServo();
  Wire.begin();
  setupRangeFinder();
  setupMotorsAndEncoders();

  leftSpeedPID.SetSampleTime(50);
  rightSpeedPID.SetSampleTime(50);
  leftSpeedPID.SetMode(MANUAL);
  rightSpeedPID.SetMode(MANUAL);
  // PID limits are 0 to 255 by default
  leftSpeedPID.SetOutputLimits(-50, 50);
  rightSpeedPID.SetOutputLimits(-50, 50);

  Serial.println(F("Setup complete"));
} // END LOOP



void loop() {

  readSerialToBuffer();
  if (newDataReceived) parseData();

  if (moveRequestReceived) {
    Serial.print(newCommandType); Serial.print('\t');
    Serial.print(nextTurnAngle); Serial.print('\t');
    Serial.print(nextMoveForward); Serial.print('\n');

    setTargets();
    clearMovementAccumulators();
    resetCounters();
    if ( abs(nextTurnAngle) > 0.01) {
      turnRoutine();
    }

    //      calcOverallMovement();
    Serial.print(F("Turn/Move:")); Serial.print('\t');
    Serial.print(angleTurned); Serial.print('\t');
    Serial.print(distanceMoved); Serial.print('\n');
    Serial.print("Total"); Serial.print('\t');
    Serial.print(theta); Serial.print('\t');
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\n');

    delay(500);
    resetCounters();

    if (abs(nextMoveForward) > 0.01) {
      forwardRoutine();
    }

    Serial.print(F("Turn/Move:")); Serial.print('\t');
    Serial.print(angleTurned); Serial.print('\t');
    Serial.print(distanceMoved); Serial.print('\n');
    Serial.print("Total"); Serial.print('\t');
    Serial.print(theta); Serial.print('\t');
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\n');
    Serial.print('\n');

    delay(500);
    calcOverallMovement();
    sendMotionData();
    clearMovementAccumulators();
    takeRangeReadings();
    sendSensorData();
    moveRequestReceived = false;
  }

} // END LOOP


//////////////////////////////////////////////////////
// FOR RANGE SENSING /////////////////////////////////
//////////////////////////////////////////////////////


void takeRangeReadings() {
  //  byte counter = 0;
  for (byte i = 0; i < numberOfReadings; i++) {
    servo.write(rangeAngles[i]);
    delay(servoDelay);
    rangeReadings[i] = rangeFinder.readRangeSingleMillimeters();
    if (rangeReadings[i] > 2000) rangeReadings[i] = 0; // above ~2000 the sensor will return ~8000
    //    Serial.println(counter);
    //    counter++;
  }
  servo.write(minAngle);  // ready for next time
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


// standard command to move a set distance and read the sensors
void processCommandType1 (char* element) {
  nextTurnAngle = defaultTurnAngle;
  nextMoveForward = defaultMoveForward;
  moveRequestReceived = true;
}

// usual command will be to turn and move a specific distance
void processCommandType2 (char* element) {
  static byte variableCounter = 0;  // expecting a maximum of 2 values
  switch (variableCounter) {
    case 0: // first variable, turn angle
      nextTurnAngle = atof(element);
      variableCounter++;
      break;
    case 1: // second variable, move distance
      nextMoveForward = atof(element);
      variableCounter = 0;
      moveRequestReceived = true; // that's everything done, so flag that we have a new move request ready to act upon
      break;
  }
}

void processCommandType3 (char* elementBuffer) {
  // not yet defined
}


//////////////////////////////////////////////////////
// FOR SENDING ///////////////////////////////////////
//////////////////////////////////////////////////////

// to Bluetooth module
void sendMotionData() {
  static int inc = 0;
  tm = millis();
  Serial3.print('1'); Serial3.print('\t');
  Serial3.print(tm); Serial3.print('\t');
  Serial3.print(inc); Serial3.print('\t');
  Serial3.print(theta, 5); Serial3.print('\t');
  Serial3.print(x, 5); Serial3.print('\t');
  Serial3.print(y, 5); Serial3.print('\n');
  inc += 1;
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
    rangeAngles[i] = i + minAngle;
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
  byte dir = bitRead(PINC, 6);
  if (dir == 1) {
    leftEncoderCounter++;
  }
  else {
    leftEncoderCounter--;
  }
}

void countRightEncoder() {
  byte dir = bitRead(PIND, 7);
  if (dir == 0) {
    rightEncoderCounter++;
  }
  else {
    rightEncoderCounter--;
  }
}

void prepareForTurn() {
    leftPWMBase = baseSpeedPWM;
    rightPWMBase = baseSpeedPWM;
    leftSpeedSetpoint = baseTurnSpeed;
    rightSpeedSetpoint = baseTurnSpeed;
    if (nextTurnAngle > 0) {
      digitalWrite(pinLeftMotorDirection, HIGH);   // anti-clockwise
      digitalWrite(pinRightMotorDirection, LOW);
    }
    else {
      digitalWrite(pinLeftMotorDirection, LOW);   // clockwise
      digitalWrite(pinRightMotorDirection, HIGH);
    }
}

void turnRoutine() {
  leftSpeedPID.SetMode(AUTOMATIC);
  rightSpeedPID.SetMode(AUTOMATIC);
  prepareForTurn();
  unsigned long timeoutStart = millis();
  speedCalcLast = millis();
  while (millis() - timeoutStart  < motorTimeout) {
    savePreviousEncoderCounts();
    copyLatestEncoderCounts();
    // quick/rough check to see if we have moved enough
    if (abs(leftEncoderCounterCopy) > abs(turnTargetTicks) || abs(rightEncoderCounterCopy) > abs(turnTargetTicks)) {
      break;
    }
    calculateSpeed();
    leftSpeedPID.Compute();
    rightSpeedPID.Compute();
    motorsTurn();
    accumulateMovement();
  }
  stopMove();
  leftSpeedPID.SetMode(MANUAL);
  rightSpeedPID.SetMode(MANUAL);
  copyLatestEncoderCounts();
  accumulateMovement(); // to capture last bit of movement
}

void forwardRoutine() {
  leftSpeedSetpoint = baseForwardSpeed;  // speed is in encoder ticks per millisecond // 0.15 is 1 rev per second
  rightSpeedSetpoint = baseForwardSpeed;
  leftSpeedPID.SetMode(AUTOMATIC);
  rightSpeedPID.SetMode(AUTOMATIC);
  unsigned long timeoutStart = millis();
  speedCalcLast = millis();
  while (millis() - timeoutStart  < motorTimeout) {
    savePreviousEncoderCounts();
    copyLatestEncoderCounts();
    // quick/rough check to see if we have moved enough
    if (abs(leftEncoderCounterCopy) > abs(distanceTargetTicks) || abs(rightEncoderCounterCopy) > abs(distanceTargetTicks)) {
      break;
    }
    calculateSpeed();
    leftSpeedPID.Compute();
    rightSpeedPID.Compute();
    motorsForward();  // update motors
    accumulateMovement();

  }
  stopMove();
  leftSpeedPID.SetMode(MANUAL);
  rightSpeedPID.SetMode(MANUAL);
  copyLatestEncoderCounts();
  accumulateMovement(); // to capture last bit of movement

}



void resetCounters() {
  cli();
  leftEncoderCounter = 0;
  rightEncoderCounter = 0;
  sei();
  leftEncoderCounterCopy = 0;
  rightEncoderCounterCopy = 0;
  leftTicksLast = 0;
  rightTicksLast = 0;
}

void motorsTurn() {
  // direction has already been set, PID is only controlling absolute speed
  leftPWM = leftPWMBase + (int)leftPWMOutput;
  rightPWM = rightPWMBase + (int)rightPWMOutput;
  analogWrite(pinLeftMotorPWM, leftPWM);
  analogWrite(pinRightMotorPWM, rightPWM);
}

void motorsForward() {
  leftPWM = baseSpeedPWM + (int)leftPWMOutput;
  rightPWM = baseSpeedPWM + (int)rightPWMOutput;
  digitalWrite(pinLeftMotorDirection, LOW);
  analogWrite(pinLeftMotorPWM, leftPWM);
  digitalWrite(pinRightMotorDirection, LOW);
  analogWrite(pinRightMotorPWM, rightPWM);
}

void stopMove() {
  digitalWrite(pinLeftMotorPWM, LOW);
  digitalWrite(pinRightMotorPWM, LOW);
}

void setTargets() {
  distanceTargetTicks = long((float)nextMoveForward / distancePerTick);
  // this is the absolute distance the LEFT wheel will need to travel (assuming right wheel moves oppositely) to acheive the desired angle
  float wheelDistanceToTravel = (nextTurnAngle / (2 * PI) ) * turningCircleCircumference;
  turnTargetTicks = (long)(wheelDistanceToTravel / distancePerTick);
}

void calculateSpeed() {
  // units of speed are ticks per millisecond
  unsigned long now = millis();
  speedCalcInterval = now - speedCalcLast;
  leftSpeedMeasured = (leftEncoderCounterCopy - leftTicksLast) / (float)speedCalcInterval;
  rightSpeedMeasured = (rightEncoderCounterCopy - rightTicksLast) / (float)speedCalcInterval;
  leftSpeedAbsolute = abs(leftSpeedMeasured);
  rightSpeedAbsolute = abs(rightSpeedMeasured);
  speedCalcLast = now;
}

void copyLatestEncoderCounts() {
  cli();
  leftEncoderCounterCopy = leftEncoderCounter;
  rightEncoderCounterCopy = rightEncoderCounter;
  sei();
}

void savePreviousEncoderCounts() {
  leftTicksLast = leftEncoderCounterCopy;
  rightTicksLast = rightEncoderCounterCopy;
}

// I have to calculate the speed anyway (to use in the PID controller) so reuse those numbers
void accumulateMovement() {
  Dl = (float)(leftEncoderCounterCopy - leftTicksLast) * distancePerTick;
  Dr = (float)(rightEncoderCounterCopy - rightTicksLast) * distancePerTick;
  Dc = (Dl + Dr) / 2.0;
  dTheta = (Dr - Dl) / wheelBase;
  x += Dc * cos(theta);
  y += Dc * sin(theta);
  theta += dTheta;
}


void calcOverallMovement() {
  // final heading is based on how much it turned overall
  angleTurned = fmod(theta, TWO_PI);
  // distance is just based on end point (don't care about path)
  distanceMoved = sqrt(x * x + y * y);  // in millimetres
}

void clearMovementAccumulators() {
  theta = 0;
  x = 0;
  y = 0;
}


// FOR DEBUGGING ////////////////////////////////////////

void printCounters() {
  cli();
  Serial.print(leftEncoderCounter); Serial.print('\t');
  Serial.print(rightEncoderCounter); Serial.print('\n');
  sei();
}




