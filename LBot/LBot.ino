// now change the data being sent to be more like an array of distance measurements

// for checking serial input
bool newInput = false;
byte input;

// for sending output
unsigned long lastSent = 0;
unsigned long tm;


// distance readings
// assume there will be 19 (from 0 to 180 degrees)
int readings[19];

// this is just a placeholder
void takeReadings() {
  for (int i = 0; i < 19; i++) {
    readings[i] = random(100);
  }
}


void setup() {
  Serial.begin(115200); // to connect to computer
  Serial1.begin(115200); // to connect to BT module
  randomSeed(analogRead(0));
}

void checkForInput() {
  if (Serial1.available()) {
    input = Serial1.read();
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
  Serial.print(tm); Serial.print('\t');
  Serial.print(inc); Serial.print('\t');
  for (int i = 0; i < 19; i++) {
    Serial.print(readings[i]);
    if (i!=18) Serial.print('\t');
  }
  Serial.print('\n');

  Serial1.print(tm); Serial1.print('\t');
  Serial1.print(inc); Serial1.print('\t');
  for (int i = 0; i < 19; i++) {
    Serial1.print(readings[i]);
    if (i!=18) Serial1.print('\t');
  }
  Serial1.print('\n');

  inc += 1;
}




void loop() {

  checkForInput();
  showInput();
  takeReadings();

  tm = millis();
  if (tm - lastSent > 2000) {
    sendData();
    lastSent += 2000;
  }
}



