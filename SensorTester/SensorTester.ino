#include <NewPing.h>


#define robotnum 2          // Robot Number (1, 2, 3)
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
#define LE 18               // Left Encoder
#define RE 19               // Right Encoder
#define RED 26              // Red Led R2
#define GREEN 22            // Green Led R2
#define YELLOW 24           // YELLOW LED R2
#define S1 4                // PIN 1 for stepper
#define S2 5                // PIN 2 for stepper
#define S3 6                // PIN 3 for stepper
#define S4 7                // PIN 4 for stepper
#define FULLSTEP 4

int LEVal = 0;
int REVal = 0;
int LELast = 0;
int RELast = 0;
int LEState = 0;
int REState = 0;

#define IR_DELAY 90000

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(RE), Rencoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LE), Lencoder, CHANGE);
  
}

void loop() {

////  int QTI1 = QTIVal(QTIsense1);
//  int QTI2 = QTIVal(QTIsense2);
  float d = pulseIn(IR_F, 0);
  float x = 1 / ((d / 100000) * 2);
  float c = pulseIn(IR_B, 0, IR_DELAY);
  float y = 1 / ((c / 100000) * 2);
  int  dist = sonar.ping_cm();

//  Serial.print(" | Front Distance: ");
//  Serial.print(dist);
  Serial.print(" | Front IR: ");
  Serial.print(x);
  Serial.print(" | Back IR: ");
  Serial.println(y);
//  Serial.print(" | QTI1: ");
//  Serial.print(QTI1);
//  Serial.print(" | QTI2: ");
//  Serial.print(QTI2);
//  Serial.print(" | LE: ");
//  Serial.print(LEVal);
//  Serial.print(" | RE: ");
//  Serial.println(REVal);
  delay(25);

}

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

void Lencoder() {
  LEVal++;
}

//============

void Rencoder() {
  REVal++;
}

