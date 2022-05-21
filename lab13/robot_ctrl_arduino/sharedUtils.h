#pragma once

// IMU related libraries
#include "ICM_20948.h"

// TOF related libraries
#include <vl53l1_error_codes.h>
#include <vl53l1x_class.h>
#include "SparkFun_VL53L1X.h"

// Matrix library
#include <BasicLinearAlgebra.h>
using namespace BLA;

// IMU sensor declaration
ICM_20948_I2C myICM;

// TOF sensor declarations
SFEVL53L1X tofSensors[2] = {
    SFEVL53L1X(),
    SFEVL53L1X(),
};

float ble_runtime_value = 0.0;
float tx_tof_f_value = 0.0;
float tx_tof_r_value = 0.0;
int motorPWM = 0;
EString tx_imu_value;
EString tx_sensor_value;

// Motor control definitions
#define LEFT 0
#define RIGHT 1
#define FORWARD 2
#define BACKWARD 3
#define STOP 4
#define ASTOP 5

void drive(int direction, int speed);

// Used to control if PID controller is running for stopping 300mm from the
// front wall
bool pid_running = false;

// Used for running the drive functions without powering the motors
bool debug_mode = false;

bool stunt_running = false;
bool spin_running = false;

void resetPID();
void driveWithKF();
void driveWithPID();
void writeSpeedToBLE();

void stunt(int* i);
void spin360(int* i, int initSpeed);
