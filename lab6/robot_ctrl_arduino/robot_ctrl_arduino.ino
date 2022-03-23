#include "bleManagement.h"
#include "motorControl.h"
#include "sensors.h"
#include "pid.h"

#define FREQ_CHECK_CONNECTION 100

void setup()
{
  // Start Serial and Wire
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // Initialize IMU and TOF sensors for reading
  setupSensors();

  // Initialize BLE for communication with Apollo
  setupBLE();

  // Initialize motor driver pins for controlling the robot
  setupMotorControl();
}

void loop()
{
  // Open up central to continuously check for BLE commands
  checkForConnections();
  delay(FREQ_CHECK_CONNECTION);
}
