#include "sharedUtils.h"

// PID constants
double kp = 1.0;
double ki = 0.0;
double kd = 0.0;

// Threshold for acceptable speed
int threshold = 50;

// Speed to drive the robot
int speed = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double pidError;
double lastError;
double input, output;
double setPoint = 300; // 300 mm from the front wall;
double cumError, rateError;

void resetPID()
{
  cumError = 0;
  rateError = 0;
  lastError = 0;
  previousTime = 0;
  elapsedTime = 0;
}

double computePID(double in)
{
  currentTime = millis();                                    // get current time
  if (previousTime == 0) {
    previousTime = millis() - 1;
  }
  elapsedTime = (double)(currentTime - previousTime) / 1000; // compute time e
  pidError = in - setPoint;                                  // determine error
  cumError += pidError * elapsedTime;                        // compute integral
  rateError = (pidError - lastError) / elapsedTime;          // compute derivative

  double out = kp * pidError + ki * cumError + kd * rateError; // PID output

  double maxErr = 5000.0;

  if (out > maxErr)
  {
    out = maxErr;
  }
  else if (out < -maxErr)
  {
    out = -maxErr;
  }

  lastError = pidError;       // remember current error
  previousTime = currentTime; // remember current time

  return out; // have function return the PID output
}

int computeSpeed(double pid)
{
  // The acceptable range of PWM (speed) values the motors can take.
  // The minPWM is determined experimentally (also known as the deadband),
  // where the robot will not move below this PWM value.
  int minPWM = 30;
  int maxPWM = 150;

  // The range of TOF sensor values. Distances cannot be negative, but the error
  // can be, since the error is the setpoint minus the (positve) sensor reading.
  // The max value is NOT as far as the sensor allows, but it is the maximum PWM
  // value the max distance should take.
  double minErr = 0.0;
  double maxErr = 5000.0;

  return (int)((maxPWM - minPWM) / (maxErr - minErr) * pid) + minPWM;
}

void driveWithPID()
{
  input = tx_tof_f_value; // Front TOF sensor reading (mm)
  output = computePID(input);
  speed = computeSpeed(output);
  if (speed > 150) {
    speed = 120;
  }

  // Control the motor based on PID value
  if (!debug_mode) {
    if (speed > threshold)
    {
      drive(FORWARD, speed);
    }
    else if (speed < -threshold)
    {
      drive(BACKWARD, speed);
    }
    else
    {
      drive(STOP, 0);
    }
  }
  Serial.print("Input (TOF): ");
  Serial.println(input);
  Serial.print("PID: ");
  Serial.println(output);
  Serial.print("Speed: ");
  Serial.println(speed);
  Serial.print("Elapsed time: ");
  Serial.println(elapsedTime);
  Serial.println("");
  delay(50);
}
