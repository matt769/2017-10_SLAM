#include <Wire.h> // Standard I2C communication library
#include <VL53L0X.h>  // laser range finder // https://github.com/pololu/vl53l0x-arduino
#include <Servo.h>    // Standard Servo library
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

#include "movement.h"

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

// for sending output
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

    delay(500);
    resetCounters();
    if (abs(nextMoveForward) > 0.01) {
      forwardRoutine();
    }

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
  for (byte i = 0; i < numberOfReadings; i++) {
    servo.write(rangeAngles[i]);
    delay(servoDelay);
    rangeReadings[i] = rangeFinder.readRangeSingleMillimeters();
    if (rangeReadings[i] > 2000) rangeReadings[i] = 0; // above ~2000 the sensor will return ~8000
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
          case 1:
            processCommandType1(elementBuffer);
            break;
          case 2:
            processCommandType2(elementBuffer);
            break;
          case 3:
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

void sendMotionData() {   // to BT module
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

void sendSensorData() {   // to BT module
  static int inc = 0;
  tm = millis();
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


// FOR DEBUGGING ////////////////////////////////////////

void printCounters() {
  cli();
  Serial.print(leftEncoderCounter); Serial.print('\t');
  Serial.print(rightEncoderCounter); Serial.print('\n');
  sei();
}




