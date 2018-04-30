
//============ Change these for each robot

#define robotnum 1        // Robot Number (1, 2, 3)
#define LF 9              // Left Motor Forward
#define LR 8              // Left Motor Reverse
#define RF 10               // Right Motor Reverse
#define RR 11              // Right  Motor Reverse
#define QTIsense1 46        // Front QTI
#define QTIsense2 3         // Back QTI
#define IR_F 44             // Front IR Sensor
#define IR_B 12             // Back IR Sensor
#define TRIGGER_PIN  2      // Front Distance Trigger Pin
#define ECHO_PIN 13         // Front Distance Echo Pin
#define MAX_DISTANCE 400    // Distance Sensor Max Distance
#define LE 32               // LE
#define RE 30               // RE
#define Red 7              // Red Led
#define Yellow 2           // Green Led
#define QTI1Match 2500   // Value greater than floor but less than black tape for QTI sensor 1 
#define QTI2Match 490    // Value greater than floor but less than black tape for QTI sensor 2

//============

int Sp = .8 * 255;      //Straight Speed limiting value to help encoders keep up
int TSp = .7 * 255;    //Turn Speed limiting value to help encoders keep up
int LEVal = 0;
int REVal = 0;
int LELast = 0;
int RELast = 0;
int LEState = 0;
int REState = 0;
int QTI1;
int QTI2;


int DesiredRobot;
int D;
int y;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];



boolean newData = false;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(LF, OUTPUT);
  pinMode(LR, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RR, OUTPUT);
  pinMode(Yellow, OUTPUT);
  pinMode(Red, OUTPUT);

  motorOff();


}


void loop() {

  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    newData = false;
  }

  if (DesiredRobot == robotnum)               // Send received values to motor
  {
    motorMapping();
    debug();
  }

  if (DesiredRobot != robotnum)               // check if this robot should not be receiving the command
  {
    motorOff();
    debug();
  }

}

//============

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

//============

void parseData() {
  // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  DesiredRobot = atoi(strtokIndx);

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  D = atoi(strtokIndx);     // convert this part to an integer

  //  strtokIndx = strtok(NULL, ",");
  //  y = atoi(strtokIndx);     // convert this part to a float

}

//============

void debug() {
  //  Serial.print("<");
  //  Serial.print(DesiredRobot);
  //  Serial.print(",");
  //  Serial.print(x);
  //  Serial.print(",");
  //  Serial.print(y);
  //  Serial.print(",");
  //  Serial.print(leftvalue);
  //  Serial.print(",");
  //  Serial.print(rightvalue);
  //  Serial.print(",");
  //  Serial.print(QTI1);
  //  Serial.print(",");
  //  Serial.print(QTI2);
  //  Serial.println(">");
}

//============

void motorMapping() {
  encoder();
  if (D == 2) {               // Forward

    if (LEVal == REVal) {
      analogWrite(LF, Sp);
      analogWrite(RF, Sp);
      analogWrite(LR, 0);
      analogWrite(RR, 0);
      debug();
    }
    if (LEVal < REVal) {
      analogWrite(LF, Sp);
      analogWrite(RF, (Sp * .8));
      analogWrite(LR, 0);
      analogWrite(RR, 0);
    }
    if (LEVal > REVal) {
      analogWrite(LF, (Sp * .8));
      analogWrite(RF, Sp);
      analogWrite(LR, 0);
      analogWrite(RR, 0);
    }
  }
  if (D == 6) {               // Reverse

    if (LEVal == REVal) {
      analogWrite(LR, Sp);
      analogWrite(RR, Sp);
      analogWrite(LF, 0);
      analogWrite(RF, 0);
      debug();
    }
    if (LEVal < REVal) {
      analogWrite(LR, Sp);
      analogWrite(RR, (Sp * .8));
      analogWrite(LF, 0);
      analogWrite(RF, 0);
    }
    if (LEVal > REVal) {
      analogWrite(LR, (Sp * .8));
      analogWrite(RR, Sp);
      analogWrite(LF, 0);
      analogWrite(RF, 0);
    }
  }
  if (D == 4) {               // Right
    if (LEVal == REVal) {
      analogWrite(LF, TSp);
      analogWrite(RR, TSp);
      analogWrite(LR, 0);
      analogWrite(RF, 0);
      debug();
    }
    if (LEVal < REVal) {
      analogWrite(LF, TSp);
      analogWrite(RR, (TSp * .8));
      analogWrite(LR, 0);
      analogWrite(RF, 0);
    }
    if (LEVal > REVal) {
      analogWrite(LF, (TSp * .8));
      analogWrite(RR, TSp);
      analogWrite(LR, 0);
      analogWrite(RF, 0);
    }
  }
  if (D == 8) {               // Left
    if (LEVal == REVal) {
      analogWrite(LR, TSp);
      analogWrite(RF, TSp);
      analogWrite(LF, 0);
      analogWrite(RR, 0);
      debug();
    }
    if (LEVal < REVal) {
      analogWrite(LR, TSp);
      analogWrite(RF, (TSp * .8));
      analogWrite(LF, 0);
      analogWrite(RR, 0);
    }
    if (LEVal > REVal) {
      analogWrite(LR, (TSp * .8));
      analogWrite(RF, TSp);
      analogWrite(LF, 0);
      analogWrite(RR, 0);
    }
  }
  if (D == 0) {               // Off
    motorOff();
  }
}

//============

void motorOff() {
  analogWrite(LF, 0);                      // turn off left motor
  analogWrite(LR, 0);                      // turn off right motor
  analogWrite(RF, 0);                      // turn off left motor
  analogWrite(RR, 0);                      // turn off right motor
  delay(10);
}

//============

void encoder() {
  LEState = digitalRead(LE);
  REState = digitalRead(RE);
  if (LEState != LELast) {
    LEVal = LEVal + 1;
  }
  if (REState != RELast) {
    REVal = REVal + 1;
  }
  LELast = LEState;
  RELast = REState;
  debug();
}

//============

void QTICheck(){
  QTI1 = QTIVal(QTIsense1);
  QTI2 = QTIVal(QTIsense2);
  debug();

  if (QTI1 < QTI1Match && QTI2 < QTI2Match)
  {
    digitalWrite(Yellow, LOW);
    digitalWrite(Red, LOW);
  }
  if (QTI1 >= QTI1Match && QTI2 < QTI2Match)
  {
    digitalWrite(Yellow, HIGH);
    digitalWrite(Red, LOW);
  }
  if (QTI2 > QTI2Match)
  {
    digitalWrite(Yellow, LOW);
    digitalWrite(Red, HIGH);
    motorOff();
    boolean stop = true;
    while (stop == true) {
      delay(100);
    }
  }
}

//============

long QTIVal(int sensorIn) {
  long duration = 0;
  pinMode(sensorIn, OUTPUT);
  digitalWrite(sensorIn, HIGH);
  delay(1);
  pinMode(sensorIn, INPUT);
  digitalWrite(sensorIn, LOW);
  while (digitalRead(sensorIn)) {
    duration++;
  }
  return duration;
}

//============


