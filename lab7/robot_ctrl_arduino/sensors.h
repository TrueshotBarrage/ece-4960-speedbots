#include "Wire.h"
#include "sharedUtils.h"

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 0      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0

// Shutdown pin for the second VL53L1X (TOF) sensor on the Artemis board
#define SHUTDOWN_PIN 4

void setupIMU()
{
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status != ICM_20948_Stat_Ok)
    {
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  
}

void setupTOF()
{
  // Turn off TOF sensor 2 by powering XSHUT signal
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);
  
  delay(200);

  // Start TOF sensor 1
  if (tofSensors[0].begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 bad init. Please check wiring.");
  }
  else
  {
    Serial.println("Sensor 1 online!");
  }

  // int a = tofSensors[0].getI2CAddress(); this is how to get the I2C address

  // Set I2C address of sensor 1 to 0x54
  tofSensors[0].setI2CAddress(0x54);
  Serial.println("Sensor 1 address set to 0x54");

  // Reactivate and start sensor 2
  digitalWrite(SHUTDOWN_PIN, HIGH);

  if (tofSensors[1].begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println("Sensor 2 bad init. Please check wiring.");
  }
  else
  {
    Serial.println("Sensor 2 online!");
  }
}

void setupSensors() {
  setupIMU();
  setupTOF();
}
