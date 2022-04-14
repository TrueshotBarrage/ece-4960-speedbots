#include "bleManagement.h"
#include "motorControl.h"
#include "sensors.h"
#include "pid.h"
#include "driveOps.h"

#define FREQ_CHECK_CONNECTION 100

void setup()
{
  // Start Serial and Wire
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Starting up");

  // Initialize IMU and TOF sensors for reading
  setupSensors();
  Serial.println("Sensors initialized");

  // Initialize BLE for communication with Apollo
  setupBLE();
  Serial.println("BLE initialized");

  // Initialize motor driver pins for controlling the robot
  setupMotorControl();
  Serial.println("Motors initialized");
}

void loop()
{
  // Open up central to continuously check for BLE commands
  checkForConnections();
  delay(FREQ_CHECK_CONNECTION);
}
