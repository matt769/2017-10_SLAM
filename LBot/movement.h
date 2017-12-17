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

void savePreviousEncoderCounts() {
  leftTicksLast = leftEncoderCounterCopy;
  rightTicksLast = rightEncoderCounterCopy;
}

void copyLatestEncoderCounts() {
  cli();
  leftEncoderCounterCopy = leftEncoderCounter;
  rightEncoderCounterCopy = rightEncoderCounter;
  sei();
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
  savePreviousEncoderCounts();
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
  savePreviousEncoderCounts();
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



