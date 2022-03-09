// #include "Wire.h"

#define A1_LT_DR A0  // Left motor driver A1
#define A2_LT_DR A1  // Left motor driver A2
#define A1_RT_DR A2  // Right motor driver A1
#define A2_RT_DR A3  // Right motor driver A2

#define LEFT 0
#define RIGHT 1
#define FORWARD 2
#define BACKWARD 3
#define STOP 4

#define CALIBRATION_FACTOR 0.4  // Right motor speed adjustment factor

void setup() {
  pinMode(A1_LT_DR, OUTPUT);
  pinMode(A2_LT_DR, OUTPUT);
  pinMode(A1_RT_DR, OUTPUT);
  pinMode(A2_RT_DR, OUTPUT);

  Serial.begin(115200);

  delay(3000);
}

void lcw(int speed) {
  analogWrite(A1_LT_DR, speed);
  analogWrite(A2_LT_DR, 0);
  Serial.print("left clockwise ");
  Serial.println(speed);
}

void rcw(int speed) {
  analogWrite(A1_RT_DR, speed);
  analogWrite(A2_RT_DR, 0);
  Serial.print("right clockwise ");
  Serial.println(speed);
}

void lccw(int speed) {
  analogWrite(A1_LT_DR, 0);
  analogWrite(A2_LT_DR, speed);
  Serial.print("left counterclockwise ");
  Serial.println(speed);
}

void rccw(int speed) {
  analogWrite(A1_RT_DR, 0);
  analogWrite(A2_RT_DR, speed);
  Serial.print("right counterclockwise ");
  Serial.println(speed);
}

void ls() {
  analogWrite(A1_LT_DR, 0);
  analogWrite(A2_LT_DR, 0);
  Serial.println("left stop");
}

void rs() {
  analogWrite(A1_RT_DR, 0);
  analogWrite(A2_RT_DR, 0);
  Serial.println("right stop");
}

void drive(int direction, int speed) {
  switch (direction) {
    case LEFT:
      lcw(speed);
      rcw(speed);
      break;
    case RIGHT:
      lccw(speed);
      rccw(speed);
      break;
    case FORWARD:
      lccw(speed);
      rcw(int(speed * CALIBRATION_FACTOR));
      break;
    case BACKWARD:
      lcw(speed);
      rccw(int(speed * CALIBRATION_FACTOR));
      break;
    case STOP:
      ls();
      rs();
      break;
  }
}

void loop() {
  drive(FORWARD, 100);
  delay(1000);
  drive(STOP, 0);
  delay(1000);
  drive(BACKWARD, 80);
  delay(1000);
  drive(STOP, 0);
  delay(1000);
  drive(LEFT, 150);
  delay(500);
  drive(STOP, 0);
  delay(1000);
  drive(RIGHT, 150);
  delay(500);
  drive(STOP, 0);
  delay(5000);
}
