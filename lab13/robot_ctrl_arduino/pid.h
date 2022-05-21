#include "sharedUtils.h"

// PID constants
double kp = 1.0;
double ki = 0.0;
double kd = 0.0;

//////// KF Variables ////////
float d_val = 4.0e-4;  // drag
float m_val = 2.61e-4; // mass

// A, B, C matrices
Matrix<2,2> A_mat = { 0, 1,
                      0, -d_val/m_val };
Matrix<2,1> B_mat = { 0, 
                      1/m_val };
Matrix<1,2> C_mat = { -1, 0 };

// Process and measurement noise
Matrix<2,2> sig_u = { 10^2, 0,
                      0, 10^2 };
Matrix<1,1> sig_z = { 20^2 };

// Discretize A & B
float delta_t = 0.2536;
Matrix<2,2> I_mat = { 1, 0,
                      0, 1     };
Matrix<2,2> A_d   = { 1, 0.254,
                      0, 0.611 };
Matrix<2,1> B_d   = { 0,
                      971.648  };

// Initial states
Matrix<2,2> sig   = { 5^2, 0,
                      0, 5^2 }; // initial state uncertainty
Matrix<2,1> x_val = { -2909, 
                      0      }; // initial state output

float e = 0;
float prev_e = 0;
float d_e = 0;
int pwm_val = 0;

//////// KF Function ////////
void kf() {
  Matrix<2,1> x_p = A_d*x_val + B_d*pwm_val;
  Matrix<2,2> sig_p = A_d*sig*(~A_d) + sig_u;

  Matrix<1,1> y_curr = { tx_tof_f_value };
  Matrix<1,1> y_m = y_curr - C_mat*x_p;
  Matrix<1,1> sig_m = C_mat*sig_p*(~C_mat) + sig_z;

  Matrix<1,1> sig_m_inv = sig_m;
  Invert(sig_m_inv);

  Matrix<2,1> kf_gain = sig_p*(~C_mat)*(sig_m_inv);

  // Update
  x_val = x_p + kf_gain*y_m;
  sig = (I_mat - kf_gain*C_mat)*sig_p;
}

void driveWithKF()
{
  kf();
  
  e = -1 * x_val(0,0) - 500;
  prev_e = e;
  d_e = e - prev_e;
  pwm_val = kp*e + kd*d_e;

  if (pwm_val > 120) {
    pwm_val = 120;
  }
  
  if (pwm_val > 1) {
    drive(FORWARD, pwm_val);
  } else if (pwm_val < -1) {
    drive(BACKWARD, pwm_val * -1);
  } else {
    drive(ASTOP, 0);
  }
  
  prev_e = e;
}

// //////////////

// Threshold for acceptable speed
int threshold = 50;

// Speed to drive the robot
int speed = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double pidError;
double lastError;
double input, output;
double setPoint = 800; // 800 mm from the front wall;
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
  int maxPWM = 200;

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
      drive(ASTOP, 0);
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
}
