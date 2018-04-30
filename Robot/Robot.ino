#include <AccelStepper.h>
#include <NewPing.h>

//============ Change these for each robot

#define robotnum 1          // Robot Number (1, 2, 3)
#define LF 9                // Left Motor Forward
#define LR 8                // Left Motor Reverse
#define RF 10               // Right Motor Reverse
#define RR 11               // Right  Motor Reverse
#define QTIsense1 46        // Front QTI
#define QTIsense2 3         // Back QTI
#define IR_F 44             // Front IR Sensor
#define IR_B 12             // Back IR Sensor
#define TRIGGER_PIN  2      // Front Distance Trigger Pin
#define ECHO_PIN 13         // Front Distance Echo Pin
#define MAX_DISTANCE 400    // Distance Sensor Max Distance
#define LE 32               // Left Encoder
#define RE 30               // Right Encoder
//#define RED 51               // Red Led
//#define GREEN 49            // Green Led
//#define YELLOW 31           // YELLOW LED
#define RED 26               // Red Led R2
#define GREEN 22            // Green Led R2
#define YELLOW 24           // YELLOW LED R2
#define S1 4                // PIN 1 for stepper
#define S2 5                // PIN 2 for stepper
#define S3 6                // PIN 3 for stepper
#define S4 7                // PIN 4 for stepper
#define FULLSTEP 4

//============ Change these based off Measured Values
#define QTI1Match 2500      // Value greater than floor but less than black tape for front QTI 
#define QTI2Match 490       // Value greater than floor but less than black tape for back QTI
#define MinDist 5           // Minimum Measurable Distance
#define LiftDist 320       // Distance Lift Box travels from top to bottom
#define IR_DELAY 100000


int Sp = .8 * 255;          // Straight Speed limiting value to help encoders keep up
int TSp = .7 * 255;         // Turn Speed limiting value to help encoders keep up

//============ Global Variables

#define motorPin1  S1     // IN1 on the ULN2003 driver 1
#define motorPin2  S2     // IN2 on the ULN2003 driver 1
#define motorPin3  S3     // IN3 on the ULN2003 driver 1
#define motorPin4  S4     // IN4 on the ULN2003 driver 1
#define FULLSTEP 4

AccelStepper stepper1(FULLSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int LEVal = 0;
int REVal = 0;
int LELast = 0;
int RELast = 0;
int LEState = 0;
int REState = 0;

int QTI1;
int QTI2;
int DesiredRobot;
int B;
int C;
int dist;
int DEBBUGER;

bool LIFT_POS = false;
bool LIFTED = false;
bool LIFT_BEGIN = false;
bool DEBUG = false;
bool FIRST = true;
bool LIFT_COMPL = false;


const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;

//============

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  
  pinMode(LF, OUTPUT);
  pinMode(LR, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RR, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);

  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(900.0);
  stepper1.setSpeed(900);
  stepper1.setCurrentPosition(0);

  motorOff();


}


void loop() {
  DataReceive();
  //  if(LIFT_COMPL == false){QTICheck();}



  if (DesiredRobot == robotnum && LIFT_COMPL == false)              // Send received values to motor
  {
    if (C > 0) {
      StageAssign();
    }
    motorMapping();
    if (DEBUG == true) {
      debug();
    }
  }

  if (DesiredRobot != robotnum && DesiredRobot > 0  && LIFT_COMPL == false)              // check if this robot should not be receiving the command
  {
    motorOff();
    if (DEBUG == true) {
      debug();
    }
  }
  if (robotnum==2 && LIFT_COMPL == true)              // Send received values to motor
  {

    motorReverseMapping();
    if (DEBUG == true) {
      debug();
    }
  }
  if (robotnum ==1 && LIFT_COMPL == true)              // Send received values to motor
  {
    motorMapping();
    if (DEBUG == true) {
      debug();
    }
  }
}

//============

void DataReceive() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    newData = false;
  }
}

//============

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char StartMarker = '<';
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

    else if (rc == StartMarker) {
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
  B = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  C = atoi(strtokIndx);     // convert this part to a float

  if (C > 0) {
    StageAssign();
  }
}
//============

void StageAssign() {
  if (C == 24) {
    Lift();
  }

  if (C == 25) {
    Lower();
  }
}

//============
void Hub(int a1, int b1) {
  Serial1.print('<');
  Serial1.print(a1);
  Serial1.print(',');
  Serial1.print(b1);
  Serial1.println('>');
  delay(25);
}

//============
void debug() {
  Serial.print(" | Robot: ");
  Serial.print(DesiredRobot);
  Serial.print(" | DPad: ");
  Serial.print(B);
  Serial.print(" | C: ");
  Serial.print(C);
  Serial.print(" | DEBUGGER: ");
  Serial.print(DEBBUGER);
  Serial.print(" | Dist: ");
  Serial.print(dist);
  Serial.print(" | Stepper: ");
  Serial.print(stepper1.currentPosition());
  Serial.print(" | LE: ");
  Serial.print(LEVal);
  Serial.print(" | RE: ");
  Serial.println(REVal);
  delay(25);
}

//============

void motorMapping() {
  encoder();
  if (B == 2) {               // Forward

    if (LEVal == REVal) {
      analogWrite(LF, Sp);
      analogWrite(RF, Sp);
      analogWrite(LR, 0);
      analogWrite(RR, 0);
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
  if (B == 6) {               // Reverse

    if (LEVal == REVal) {
      analogWrite(LR, Sp);
      analogWrite(RR, Sp);
      analogWrite(LF, 0);
      analogWrite(RF, 0);
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
  if (B == 4) {               // Right
    if (LEVal == REVal) {
      analogWrite(LF, TSp);
      analogWrite(RR, TSp);
      analogWrite(LR, 0);
      analogWrite(RF, 0);
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
  if (B == 8) {               // Left
    if (LEVal == REVal) {
      analogWrite(LR, TSp);
      analogWrite(RF, TSp);
      analogWrite(LF, 0);
      analogWrite(RR, 0);
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
  if (B == 0) {               // Off
    motorOff();
  }
  encoder();
  if (DEBUG == true) {
    debug();
  }
}

//============

void motorReverseMapping() {
  encoder();
  if (B == 6) {               // Forward

    if (LEVal == REVal) {
      analogWrite(LF, Sp);
      analogWrite(RF, Sp);
      analogWrite(LR, 0);
      analogWrite(RR, 0);
    }
    if (LEVal > REVal) {
      analogWrite(LF, Sp);
      analogWrite(RF, (Sp * .8));
      analogWrite(LR, 0);
      analogWrite(RR, 0);
    }
    if (LEVal < REVal) {
      analogWrite(LF, (Sp * .8));
      analogWrite(RF, Sp);
      analogWrite(LR, 0);
      analogWrite(RR, 0);
    }
  }
  if (B == 2) {               // Reverse

    if (LEVal == REVal) {
      analogWrite(LR, Sp);
      analogWrite(RR, Sp);
      analogWrite(LF, 0);
      analogWrite(RF, 0);
    }
    if (LEVal > REVal) {
      analogWrite(LR, Sp);
      analogWrite(RR, (Sp * .8));
      analogWrite(LF, 0);
      analogWrite(RF, 0);
    }
    if (LEVal < REVal) {
      analogWrite(LR, (Sp * .8));
      analogWrite(RR, Sp);
      analogWrite(LF, 0);
      analogWrite(RF, 0);
    }
  }
  if (B == 8) {               // Right
    if (LEVal == REVal) {
      analogWrite(LF, TSp);
      analogWrite(RR, TSp);
      analogWrite(LR, 0);
      analogWrite(RF, 0);
    }
    if (LEVal > REVal) {
      analogWrite(LF, TSp);
      analogWrite(RR, (TSp * .8));
      analogWrite(LR, 0);
      analogWrite(RF, 0);
    }
    if (LEVal < REVal) {
      analogWrite(LF, (TSp * .8));
      analogWrite(RR, TSp);
      analogWrite(LR, 0);
      analogWrite(RF, 0);
    }
  }
  if (B == 6) {               // Left
    if (LEVal == REVal) {
      analogWrite(LR, TSp);
      analogWrite(RF, TSp);
      analogWrite(LF, 0);
      analogWrite(RR, 0);
    }
    if (LEVal > REVal) {
      analogWrite(LR, TSp);
      analogWrite(RF, (TSp * .8));
      analogWrite(LF, 0);
      analogWrite(RR, 0);
    }
    if (LEVal < REVal) {
      analogWrite(LR, (TSp * .8));
      analogWrite(RF, TSp);
      analogWrite(LF, 0);
      analogWrite(RR, 0);
    }
  }
  if (B == 0) {               // Off
    motorOff();
  }
  encoder();
  if (DEBUG == true) {
    debug();
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
}

//============

void QTICheck() {
  QTI1 = QTIVal(QTIsense1);
  QTI2 = QTIVal(QTIsense2);
  if (DEBUG == true) {
    debug();
  }

  if (QTI1 < QTI1Match && QTI2 < QTI2Match)
  {
    digitalWrite(RED, LOW);
  }
  if (QTI1 >= QTI1Match && QTI2 < QTI2Match)
  {
    digitalWrite(RED, LOW);
  }
  if (QTI2 > QTI2Match)
  {
    digitalWrite(RED, HIGH);
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

void Distance() {
  dist = sonar.ping_cm();
  if (dist == 0) {
    dist = MAX_DISTANCE;
  }
  if (DEBUG == true) {
    debug();
  }
}

//============
void IR() {
  float d = pulseIn(IR_F, 0, IR_DELAY);
  float x = 1 / ((d / 1000000) * 2);
  //  float c = pulseIn(IR_B, 0, IR_DELAY);
  //  float y = 1 / ((c / 1000000) * 2);

  if (x >= 11 && x <= 12) {
    digitalWrite(GREEN, HIGH);
    digitalWrite(YELLOW, LOW);
  }

  else if (x >= .1 && x < 11 || x > 12) {
    digitalWrite(GREEN, LOW);
    digitalWrite(YELLOW, HIGH);
  }

  else {
    digitalWrite(GREEN, LOW);
    digitalWrite(YELLOW, LOW);
  }


}
//============

void Lift() {
  digitalWrite(GREEN, HIGH);
  stepper1.move(LiftDist);
  stepper1.runToPosition();
  digitalWrite(GREEN, LOW);
  LIFT_COMPL = true;
}

//============


void Lower() {
  stepper1.move(-LiftDist);
  stepper1.runToPosition();
  stepper1.run();

}

