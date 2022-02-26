/****************************************************************
   Example1_Basics.ino
   ICM 20948 Arduino Library Demo
   Use the default configuration to stream 9-axis IMU data
   Owen Lyke @ SparkFun Electronics
   Original Creation Date: April 17 2019

   Please see License.md for the license information.

   Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h"

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 0      // The value of the last bit of the I2C address.                \
  // On the SparkFun 9DoF IMU breakout the default is 1, and when \
  // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

void setup()
{

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(80000);
#endif
  // Uncomment this line to enable helpful debug messages on Serial
  //myICM.enableDebugging(); 

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    //    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    //    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}

void loop()
{

  if (myICM.dataReady())
  {
    // The values are only updated when you call 'getAGMT'
    myICM.getAGMT();
    
    // Uncomment this to see the raw values, taken directly from the agmt structure
    // printRawAGMT( myICM.agmt );

    // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    printScaledAGMT(&myICM);

    // This allows us to print just the roll and pitch from the accelerometer data
    // printAccelRollAndPitch(&myICM);

    // This allows us to print roll, pitch, and yaw from the gyroscope data
    // printGyroRollPitchYaw(&myICM);
    
    delay(30);
  }
  else
  {
    //    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }
}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
//    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  //  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  SERIAL_PORT.print("accX:");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", accY:");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", accZ:");
  printFormattedFloat(sensor->accZ(), 5, 2);
  //  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  SERIAL_PORT.print(", gyrX:");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", gyrY:");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", gyrZ:");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  //  SERIAL_PORT.print(" ], Mag (uT) [ ");
  SERIAL_PORT.print(", magX:");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", magY:");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", magZ:");
  printFormattedFloat(sensor->magZ(), 5, 2);
  //  SERIAL_PORT.print(" ], Tmp (C) [ ");
  SERIAL_PORT.print(", temp:");
  printFormattedFloat(sensor->temp(), 5, 2);
  //  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

#ifdef USE_SPI
void printAccelRollAndPitch(ICM_20948_SPI *sensor)
{
#else
void printAccelRollAndPitch(ICM_20948_I2C *sensor)
{
#endif
  float roll = atan2(sensor->accY(), sensor->accZ());
  SERIAL_PORT.print("roll:");
  printFormattedFloat(roll, 5, 2);

  float pitch = atan2(sensor->accX(), sensor->accZ());
  SERIAL_PORT.print(", pitch:");
  printFormattedFloat(pitch, 5, 2);
  SERIAL_PORT.println();
}

#ifdef USE_SPI
void printGyroRollPitchYaw(ICM_20948_SPI *sensor)
{
#else
void printGyroRollPitchYaw(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("gyrX:");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", gyrY:");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", gyrZ:");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.println();
}
