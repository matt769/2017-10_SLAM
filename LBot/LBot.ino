

#include <Wire.h>
#include <VL53L0X.h>  // laser range finder
#include <Servo.h>
#include <PID_v1.h>

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
const float wheelBase = 117.0; // in millimeters // placeholder
const float wheelCircumference = 60.0 * PI; // in millimeters // placeholder
float angleTurned;
float distanceMoved;
const int encoderTicksPerRevolution = 7 * 100;  // ticks per motor shaft turn * gear ratio
const float distancePerTick = wheelCircumference / (float)encoderTicksPerRevolution;
long distanceTargetTicks = 0;  // after receiving a distance to travel, convert to encoder ticks for quicker calculation
long turnTargetTicks = 0;
float turningCircleCircumference = wheelBase * PI;
bool inMotionForward = false;
bool inMotionTurning = false;

// for motors
const byte pinLeftMotorDirection = 4;
const byte pinRightMotorDirection = 5;
const byte pinLeftMotorPWM = 10;
const byte pinRightMotorPWM = 11;
const byte pinLeftEncoderA = 2;
const byte pinLeftEncoderB = 17;
const byte pinRightEncoderA = 3;
const byte pinRightEncoderB = 9;
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


float leftSpeedSetpoint, leftSpeedActual, leftPWMOutput, rightSpeedSetpoint, rightSpeedActual, rightPWMOutput;
float Kp = 500, Ki = 0, Kd = 0;
PID leftSpeedPID(&leftSpeedActual, &leftPWMOutput, &leftSpeedSetpoint, Kp, Ki, Kd, DIRECT);
PID rightSpeedPID(&rightSpeedActual, &rightPWMOutput, &rightSpeedSetpoint, Kp, Ki, Kd, DIRECT);
unsigned long speedCalcLast;
long leftTicksLast;
long rightTicksLast;

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
    //    Serial.print(turnTargetTicks); Serial.print('\t');
    //    Serial.print(distanceTargetTicks); Serial.print('\n');
    resetCounters();
    //    Serial.println("1");
    turnRoutine();
    //    Serial.println("2");
    calculateTurn();
    //    Serial.println("3");
    //    Serial.print(leftEncoderCounter); Serial.print('\t');
    //    Serial.print(rightEncoderCounter); Serial.print('\t');
    Serial.print(F("Angle turned:")); Serial.print('\t');
    Serial.print(angleTurned); Serial.print('\n');
    resetCounters();

    delay(500);
    resetCounters();
    forwardRoutine();
    calculateDistance();
    //    Serial.print(leftEncoderCounter); Serial.print('\t');
    //    Serial.print(rightEncoderCounter); Serial.print('\t');
    Serial.print(F("Distance moved:")); Serial.print('\t');
    Serial.print(distanceMoved); Serial.print('\n');
    Serial.print('\n');

    sendMotionData();
    takeRangeReadings();
    sendSensorData();
    moveRequestReceived = false;
  }

} // END LOOP


//////////////////////////////////////////////////////
// FOR RANGE SENSING /////////////////////////////////
//////////////////////////////////////////////////////


void takeRangeReadings() {
  for (byte i = 0; i < numberOfReadings; i++) {
    servo.write(rangeAngles[i]);
    delay(50);
    rangeReadings[i] = rangeFinder.readRangeSingleMillimeters();
    if (rangeReadings[i] > 2000) rangeReadings[i] = 0; // above ~2000 the sensor will return ~8000
  }
  servo.write(rangeAngles[0]);  // ready for next time
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


void prepareForTurn() {
  if (nextTurnAngle == 0.0) {
    leftPWMBase = 0;
    rightPWMBase = 0;
  }
  else {
    leftPWMBase = baseSpeedPWM;
    rightPWMBase = baseSpeedPWM;
    leftSpeedSetpoint = baseTurnSpeed;
    rightSpeedSetpoint = baseTurnSpeed;

    if (nextTurnAngle > 0) {
      digitalWrite(pinLeftMotorDirection, HIGH);
      digitalWrite(pinRightMotorDirection, LOW);
    }
    else {
      digitalWrite(pinLeftMotorDirection, LOW);
      digitalWrite(pinRightMotorDirection, HIGH);
    }
  }
}

void turnRoutine() {
  leftSpeedPID.SetMode(AUTOMATIC);
  rightSpeedPID.SetMode(AUTOMATIC);
  prepareForTurn();
  motorsTurn();
  speedCalcLast = millis();
  unsigned long timeoutStart = millis();
  while (millis() - timeoutStart  < motorTimeout) {
    calculateSpeed();
    leftSpeedPID.Compute();
    rightSpeedPID.Compute();
    motorsTurn();
    cli();
    leftEncoderCounterCopy = leftEncoderCounter;
    rightEncoderCounterCopy = rightEncoderCounter;
    sei();
    if (abs(leftEncoderCounterCopy) > abs(turnTargetTicks)) { // need to account for negative
      stopMove();
      break;
    }
  }
  leftSpeedPID.SetMode(MANUAL);
  rightSpeedPID.SetMode(MANUAL);
}

void forwardRoutine() {
  leftSpeedSetpoint = baseForwardSpeed;  // speed is in encoder ticks per millisecond // 0.15 is 1 rev per second
  rightSpeedSetpoint = baseForwardSpeed;
  leftSpeedPID.SetMode(AUTOMATIC);
  rightSpeedPID.SetMode(AUTOMATIC);
  motorsForward();
  speedCalcLast = millis();
  unsigned long timeoutStart = millis();
  while (millis() - timeoutStart  < motorTimeout) {
    calculateSpeed();
    leftSpeedPID.Compute();
    rightSpeedPID.Compute();
    motorsForward();  // to update the motors
    cli();
    long leftEncoderCounterCopy = leftEncoderCounter;
    sei();
    if (abs(leftEncoderCounter) > abs(distanceTargetTicks)) {
      stopMove();
      break;
    }
  }
  leftSpeedPID.SetMode(MANUAL);
  rightSpeedPID.SetMode(MANUAL);
}

void printCounters() {
  cli();
  Serial.print(leftEncoderCounter); Serial.print('\t');
  Serial.print(rightEncoderCounter); Serial.print('\n');
  sei();
}

void resetCounters() {
  cli();
  leftEncoderCounter = 0;
  rightEncoderCounter = 0;
  sei();
}

void motorsTurn() {
  // direction has already been set
  // PID is only controlling absolute speed
  leftPWM = leftPWMBase + (byte)leftPWMOutput;
  rightPWM = rightPWMBase + (byte)rightPWMOutput;
  analogWrite(pinLeftMotorPWM, leftPWM);
  analogWrite(pinRightMotorPWM, rightPWM);
}

void motorsForward() {
  leftPWM = baseSpeedPWM + (byte)leftPWMOutput;
  rightPWM = baseSpeedPWM + (byte)rightPWMOutput;
  digitalWrite(pinLeftMotorDirection, HIGH);
  analogWrite(pinLeftMotorPWM, leftPWM);
  digitalWrite(pinRightMotorDirection, HIGH);
  analogWrite(pinRightMotorPWM, rightPWM);
}

void stopMove() {
  digitalWrite(pinLeftMotorPWM, LOW);
  digitalWrite(pinRightMotorPWM, LOW);
}

void setTargets() {
  distanceTargetTicks = long((float)nextMoveForward / distancePerTick);
  // target for the LEFT WHEEL
  // this is the absolute distance the left wheel will need to travel (assuming right wheel move oppositely) to acheive the desired angle
  float wheelDistanceToTravel = (nextTurnAngle / (2 * PI) ) * turningCircleCircumference;
  turnTargetTicks = (long)(wheelDistanceToTravel / distancePerTick);
}

void calculateTurn() {
  // for the moment assume that motors have moved the exact same (but opposite) distance
  cli();
  leftEncoderCounterCopy = leftEncoderCounter;
  rightEncoderCounterCopy = rightEncoderCounter;
  sei();
  float turnDistance = (float)leftEncoderCounterCopy * distancePerTick;
  // (turnDistance / turningCircleCircumference) gives the proportion of the full turning circle circumference that the wheel has travelled
  angleTurned = (turnDistance / turningCircleCircumference) * 2 * PI;
}

void calculateDistance() {
  // copy encoder values and reset
  // turn off interupts while doing so (the robot should not be moving at this point)
  cli();
  leftEncoderCounterCopy = leftEncoderCounter;
  rightEncoderCounterCopy = rightEncoderCounter;
  sei();
  //  Serial.print(leftEncoderCounterCopy);Serial.print('\t');
  //  Serial.print(rightEncoderCounterCopy);Serial.print('\n');
  // for the moment, assume that it has travelled in perfectly straight line
  // in which case the encoder counts will be the same
  distanceMoved = (float)leftEncoderCounterCopy * distancePerTick;
}


void calculateSpeed() {
  unsigned long interval = millis() - speedCalcLast;
  if (interval >= 100) {
    // copy encoder values and reset
    cli();
    leftEncoderCounterCopy = leftEncoderCounter;
    rightEncoderCounterCopy = rightEncoderCounter;
    sei();
    leftSpeedActual = abs((leftEncoderCounterCopy - leftTicksLast) / (float)interval);
    rightSpeedActual = abs((rightEncoderCounterCopy - rightTicksLast) / (float)interval);
    leftTicksLast = leftEncoderCounterCopy;
    rightTicksLast = rightEncoderCounterCopy;
    //    Serial.print(leftEncoderCounterCopy); Serial.print('\t');
    //    Serial.print(rightEncoderCounterCopy); Serial.print('\t');
    speedCalcLast += 100;
    //    Serial.print(leftSpeedActual); Serial.print('\t');
    //    Serial.print(rightSpeedActual); Serial.print('\n');
  }
}
