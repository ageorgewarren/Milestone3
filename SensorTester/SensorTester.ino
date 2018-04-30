#include <NewPing.h>

#define robotnum = 1     // Robot Number (1, 2, 3)
#define QTIsense1 46     // Front QTI
#define QTIsense2 3      // Back QTI
#define IR_F 44          // Front IR Sensor
#define IR_B 12          // Back IR Sensor
#define TRIGGER_PIN  2   // Front Distance Trigger Pin
#define ECHO_PIN 13      // Front Distance Echo Pin
#define MAX_DISTANCE 400 // Distance Sensor Max Distance
#define LE 32            // LE
#define RE 30            // RE

int LEVal = 0;
int REVal = 0;
int LELast = 0;
int RELast = 0;
int LEState = 0;
int REState = 0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);

}

void loop() {
  encoder();
  int QTI1 = QTIVal(QTIsense1);
  int QTI2 = QTIVal(QTIsense2);
  float d = pulseIn(IR_F, 0, 12500);
  float x = 1 / ((d / 1000000) * 2);
  float c = pulseIn(IR_B, 0, 12500);
  float y = 1 / ((c / 1000000) * 2);
  int  dist = sonar.ping_cm();

  Serial.print(" | Front Distance: ");
  Serial.print(dist);
  Serial.print(" | Front IR: ");
  Serial.print(x);
  Serial.print(" | Back IR: ");
  Serial.print(y);
  Serial.print(" | QTI1: ");
  Serial.print(QTI1);
  Serial.print(" | QTI2: ");
  Serial.print(QTI2);
  Serial.print(" | LE: ");
  Serial.print(LEVal);
  Serial.print(" | RE: ");
  Serial.println(REVal);
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
