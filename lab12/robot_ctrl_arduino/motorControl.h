#include "sharedUtils.h"

//#define A1_LT_DR A0 // Left motor driver A1
//#define A2_LT_DR A1 // Left motor driver A2
//#define A1_RT_DR A2 // Right motor driver A1
//#define A2_RT_DR A3 // Right motor driver A2

// These are Kirstin's car values!
#define A1_LT_DR A5 // Left motor driver A1
#define A2_LT_DR 4 // Left motor driver A2
#define A1_RT_DR 7 // Right motor driver A1
#define A2_RT_DR 6 // Right motor driver A2

#define CALIBRATION_FACTOR 0.85 // Right motor speed adjustment factor

void setupMotorControl()
{
  pinMode(A1_LT_DR, OUTPUT);
  pinMode(A2_LT_DR, OUTPUT);
  pinMode(A1_RT_DR, OUTPUT);
  pinMode(A2_RT_DR, OUTPUT);
}

void lcw(int speed)
{
  analogWrite(A1_LT_DR, speed);
  analogWrite(A2_LT_DR, 0);
  
  Serial.print("left clockwise ");
  Serial.println(speed);
}

void rcw(int speed)
{
  analogWrite(A1_RT_DR, speed);
  analogWrite(A2_RT_DR, 0);
  Serial.print("right clockwise ");
  Serial.println(speed);
}

void lccw(int speed)
{
  analogWrite(A1_LT_DR, 0);
  analogWrite(A2_LT_DR, speed);
  Serial.print("left counterclockwise ");
  Serial.println(speed);
}

void rccw(int speed)
{
  analogWrite(A1_RT_DR, 0);
  analogWrite(A2_RT_DR, speed);
  Serial.print("right counterclockwise ");
  Serial.println(speed);
}

void ls()
{
  analogWrite(A1_LT_DR, 0);
  analogWrite(A2_LT_DR, 0);
  Serial.println("left stop");
}

void rs()
{
  analogWrite(A1_RT_DR, 0);
  analogWrite(A2_RT_DR, 0);
  Serial.println("right stop");
}

void activeStop()
{
  analogWrite(A1_LT_DR, 255);
  analogWrite(A2_LT_DR, 255);
  analogWrite(A1_RT_DR, 255);
  analogWrite(A2_RT_DR, 255);
  Serial.println("active stop");
}

void drive(int direction, int speed)
{
  switch (direction)
  {
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
  case ASTOP:
    activeStop();
    break;
  }
  motorPWM = speed;
  writeSpeedToBLE();
}

void testDrive()
{
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
