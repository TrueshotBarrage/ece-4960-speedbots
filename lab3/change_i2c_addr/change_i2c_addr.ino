#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define SHUTDOWN_PIN 4
//#define INTERRUPT_PIN 3

SFEVL53L1X sensors[2] = {
  SFEVL53L1X(),
  SFEVL53L1X(),
};

void setup() {
  // Turn off TOF sensor 2 by powering XSHUT signal
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, HIGH);
  
  delay(200);
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (sensors[0].begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 1 online!");

  int s1Addr = sensors[0].getI2CAddress();
  Serial.print("Sensor 1: ");
  Serial.println(s1Addr);

  // Set I2C address of sensor 1 to 0x54
  sensors[0].setI2CAddress(0x54);
  Serial.println("Sensor 1 address set to 0x54");

  // Reactivate sensor 2
  digitalWrite(SHUTDOWN_PIN, LOW);
  
  s1Addr = sensors[0].getI2CAddress();
  int s2Addr = sensors[1].getI2CAddress();
  
  Serial.print("Sensor 1: ");
  Serial.println(s1Addr);
  Serial.print("Sensor 2: ");
  Serial.println(s2Addr);
}

void loop() {
  for (int i = 0; i < 2; i++) {
    SFEVL53L1X sensor = sensors[i];
    Serial.print("Sensor ");
    Serial.println(i+1);
    // Write configuration bytes to initiate measurement
    sensor.startRanging();
    
    while (!sensor.checkForDataReady())
    {
      delay(1);
    }
    // Get the result of the measurement from the sensor
    int distance = sensor.getDistance(); 
    sensor.clearInterrupt();
    sensor.stopRanging();
  
    Serial.print("Distance(mm): ");
    Serial.print(distance);
  
    float distanceInches = distance * 0.0393701;
    float distanceFeet = distanceInches / 12.0;
  
    Serial.print("\tDistance(ft): ");
    Serial.println(distanceFeet, 2);
  }
}
