#include <ArduinoBLE.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include "sharedUtils.h"

//////////// BLE UUIDs ////////////
#define BLE_UUID_SERVICE "4A7C2101-7D65-4814-81A2-7DF8ECF23609"
#define BLE_UUID_RX_CMD "9750F60B-9C9C-4158-B620-02EC9521CD99"
#define BLE_UUID_TX_TIME "715C9B74-A522-42C6-97E1-28ECF033BC9B"

#define BLE_UUID_TX_TOF_RIGHT "2F5C6DCC-0121-4B44-90F4-4685ACB8E181"
#define BLE_UUID_TX_TOF_FRONT "07565624-13EF-4780-AFEE-BD3B38082505"
#define BLE_UUID_TX_IMU "213290BB-FC80-4269-90B7-65CE2155DEE4"
#define BLE_UUID_TX_SENSOR "303FCA3B-E528-4FFD-AEE0-B531AC1670E0"
#define BLE_UUID_TX_SPEED "F3814C37-D7F0-4E48-AE13-A07A8B163B6C"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService bleService(BLE_UUID_SERVICE);
BLECStringCharacteristic cmd_string(BLE_UUID_RX_CMD, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic runtime(BLE_UUID_TX_TIME, BLERead | BLENotify);
BLEFloatCharacteristic tx_tof_r(BLE_UUID_TX_TOF_RIGHT, BLERead | BLENotify);
BLEFloatCharacteristic tx_tof_f(BLE_UUID_TX_TOF_FRONT, BLERead | BLENotify);
BLECStringCharacteristic tx_imu(BLE_UUID_TX_IMU, BLERead | BLENotify,
                                MAX_MSG_SIZE);
BLECStringCharacteristic tx_sensor(BLE_UUID_TX_SENSOR, BLERead | BLENotify, 
                                MAX_MSG_SIZE);
BLEIntCharacteristic tx_speed(BLE_UUID_TX_SPEED, BLERead | BLENotify);

// RX - receive
RobotCommand robot_cmd(":|");

// TX - transmit
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
//////////// Global Variables ////////////
int initSpeed = 0;

enum CommandTypes
{
  GET_IMU_DATA,
  GET_TOF_DATA,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  MOVE_LEFT,
  MOVE_RIGHT,
  MOVE_STOP,
  MOVE_ASTOP,
  START_PID_300MM,
  STOP_PID_300MM,
  TOGGLE_DEBUG,
  STUNT,
  SPIN_360,
};

void appendWithComma(EString *str, float val)
{
  if (str->get_length() > 0)
  {
    str->append(",");
  }
  str->append(val);
}

EString packIMUDataIntoEString(ICM_20948_I2C *imu)
{
  EString data;
  data.clear();

  float accX = imu->accX();
  float accY = imu->accY();
  float accZ = imu->accZ();
  float gyrX = imu->gyrX();
  float gyrY = imu->gyrY();
  float gyrZ = imu->gyrZ();
  float magX = imu->magX();
  float magY = imu->magY();
  float magZ = imu->magZ();
  float temp = imu->temp();

  appendWithComma(&data, accX);
  appendWithComma(&data, accY);
  appendWithComma(&data, accZ);
  appendWithComma(&data, gyrX);
  appendWithComma(&data, gyrY);
  appendWithComma(&data, gyrZ);
  appendWithComma(&data, magX);
  appendWithComma(&data, magY);
  appendWithComma(&data, magZ);
  appendWithComma(&data, temp);

  return data;
}

EString packSensorDataIntoEString(ICM_20948_I2C *imu, float tof1, float tof2)
{
  EString data;
  data.clear();

  float accX = imu->accX();
  float accY = imu->accY();
  float accZ = imu->accZ();
  float gyrX = imu->gyrX();
  float gyrY = imu->gyrY();
  float gyrZ = imu->gyrZ();
  float magX = imu->magX();
  float magY = imu->magY();
  float magZ = imu->magZ();
  float temp = imu->temp();

  // TOF
  appendWithComma(&data, tof1);
  appendWithComma(&data, tof2);

  // IMU
  appendWithComma(&data, accX);
  appendWithComma(&data, accY);
  appendWithComma(&data, accZ);
  appendWithComma(&data, gyrX);
  appendWithComma(&data, gyrY);
  appendWithComma(&data, gyrZ);
  appendWithComma(&data, magX);
  appendWithComma(&data, magY);
  appendWithComma(&data, magZ);
  appendWithComma(&data, temp);

  return data;
}

void writeIMUDataToBLE()
{
  // Update IMU sensor readings
  myICM.getAGMT();

  // Write IMU data into a string, separated by commas
  tx_imu_value = packIMUDataIntoEString(&myICM);
  Serial.print("IMU: ");
  Serial.println(tx_imu_value.c_str());

  // Write the IMU data string to the characteristic
  tx_imu.writeValue(tx_imu_value.c_str());
}

float getTOFData(int tofId)
{
  // Specify the sensor as there are multiple
  SFEVL53L1X sensor = tofSensors[tofId];

  // Write configuration bytes to initiate measurement
  sensor.startRanging();
  while (!sensor.checkForDataReady())
  {
    delay(1);
  }

  // Get the data (in mm) from the TOF sensor
  int distance = sensor.getDistance();
  sensor.clearInterrupt();
  sensor.stopRanging();

  // Other conversions to use as needed
  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  return distance;
}

void writeTOFDataToBLE()
{
  // Update ToF sensor readings
  tx_tof_f_value = getTOFData(0);
  Serial.print("TOF Front: ");
  Serial.println(tx_tof_f_value);

  tx_tof_r_value = getTOFData(1);
  Serial.print("TOF Right: ");
  Serial.println(tx_tof_r_value);

  // Write the readings into the appropriate characteristics
  tx_tof_f.writeValue(tx_tof_f_value);
  tx_tof_r.writeValue(tx_tof_r_value);
}

void writeSensorDataToBLE()
{
  // TOF
  tx_tof_f_value = getTOFData(0);
  tx_tof_r_value = getTOFData(1);

  // IMU
  myICM.getAGMT();

  // Write sensor data into a string, separated by commas - TOFR, TOFF, IMU in order
  tx_sensor_value = packSensorDataIntoEString(&myICM, tx_tof_r_value, tx_tof_f_value);

  // Write the sensor data string to the characteristic
  tx_sensor.writeValue(tx_sensor_value.c_str());

  // Write the TOF readings into the appropriate characteristics
  tx_tof_f.writeValue(tx_tof_f_value);
  tx_tof_r.writeValue(tx_tof_r_value);

  // Write IMU data into a string, separated by commas
  tx_imu_value = packIMUDataIntoEString(&myICM);

  // Write the IMU data string to the characteristic
  tx_imu.writeValue(tx_imu_value.c_str());
}

void writeSpeedToBLE()
{
  tx_speed.writeValue(motorPWM);
}

void handleCommand()
{
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(cmd_string.value(), cmd_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
   * since it uses strtok internally (refer RobotCommand.h and
   * https://www.cplusplus.com/reference/cstring/strtok/)
   */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success)
  {
    return;
  }

  // Handle the command type accordingly
  switch (cmd_type)
  {
  /*
   * Read the IMU sensor data and store it in a characteristic.
   */
  case GET_IMU_DATA:
  {
    // Read the IMU sensor data and send it to Apollo BLE
    writeIMUDataToBLE();
    Serial.println("Wrote IMU data to BLE");
    break;
  }

  /*
   * Read the TOF sensor data and store them in characteristic values.
   */
  case GET_TOF_DATA:
  {
    // Read the TOF sensor data and send it to Apollo BLE
    writeTOFDataToBLE();
    Serial.println("Wrote TOF data to BLE");
    break;
  }

  /*
   * Move the robot forward at a given speed.
   */
  case MOVE_FORWARD:
  {
    int speed;
    success = robot_cmd.get_next_value(speed);
    if (!success)
      return;

    Serial.print("Driving forward at speed (PWM) value: ");
    Serial.println(speed);

    drive(FORWARD, speed);
    break;
  }

  /*
   * Move the robot backward at a given speed.
   */
  case MOVE_BACKWARD:
  {
    int speed;
    success = robot_cmd.get_next_value(speed);
    if (!success)
      return;

    Serial.print("Driving backward at speed (PWM) value: ");
    Serial.println(speed);

    drive(BACKWARD, speed);
    break;
  }

  /*
   * Rotate left (counterclockwise) in place at a given speed.
   */
  case MOVE_LEFT:
  {
    int speed;
    success = robot_cmd.get_next_value(speed);
    if (!success)
      return;

    Serial.print("Rotating left at speed (PWM) value: ");
    Serial.println(speed);

    drive(LEFT, speed);
    break;
  }

  /*
   * Rotate right (clockwise) in place at a given speed.
   */
  case MOVE_RIGHT:
  {
    int speed;
    success = robot_cmd.get_next_value(speed);
    if (!success)
      return;

    Serial.print("Rotating right at speed (PWM) value: ");
    Serial.println(speed);

    drive(RIGHT, speed);
    break;
  }

  /*
   * Stop the robot from moving.
   */
  case MOVE_STOP:
  {
    Serial.println("Stopping movement");
    drive(STOP, 0);
    break;
  }

  /*
   * Actively stop the robot from moving.
   */
  case MOVE_ASTOP:
  {
    Serial.println("Actively stopping movement");
    drive(ASTOP, 0);
    break;
  }

  case START_PID_300MM:
  {
    Serial.println("Starting PID control to stop 300mm from forward wall");
    resetPID();
    pid_running = true;
    break;
  }

  case STOP_PID_300MM:
  {
    Serial.println("Stopping PID control");
    drive(STOP, 0);
    resetPID();
    pid_running = false;
    break;
  }

  case TOGGLE_DEBUG:
  {
    Serial.print("Toggling debug mode: ");
    debug_mode = !debug_mode;
    if (debug_mode)
    {
      Serial.println("ON");
    }
    else
    {
      Serial.println("OFF");
    }
    break;
  }

  case STUNT:
  {
    Serial.println("Stunt mode activated");
    stunt_running = true;
    break;
  }

  case SPIN_360:
  { 
    success = robot_cmd.get_next_value(initSpeed);
    if (!success)
      return;

    Serial.println("Spinning 360 degrees");
    spin_running = true;
    break;
  }

  /*
   * The default case may not capture all types of invalid commands.
   * It is safer to validate the command string on the central device (in
   * python) before writing to the characteristic.
   */
  default:
  {
    Serial.print("Invalid Command Type: ");
    Serial.println(cmd_type);
    break;
  }
  }
}

void setupBLE()
{
  Serial.begin(115200);
  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Apollo BLE");
  BLE.setLocalName("Apollo BLE");
  BLE.setAdvertisedService(bleService);

  // Add BLE characteristics
  bleService.addCharacteristic(cmd_string);
  bleService.addCharacteristic(runtime);

  bleService.addCharacteristic(tx_tof_r);
  bleService.addCharacteristic(tx_tof_f);
  bleService.addCharacteristic(tx_imu);
  bleService.addCharacteristic(tx_sensor);
  bleService.addCharacteristic(tx_speed);

  // Add BLE service
  BLE.addService(bleService);

  // Set initial values to prevent errors when reading for the first time on
  // central devices
  runtime.setValue(ble_runtime_value);

  // Output MAC Address
  Serial.print("[Apollo] Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();
}

void trackRuntime()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {
    ble_runtime_value = ble_runtime_value + 0.5;
    runtime.writeValue(ble_runtime_value);

    previousMillis = currentMillis;
  }
}

void readCommand()
{
  // Query if the characteristic value has been written by another BLE device
  if (cmd_string.written())
  {
    handleCommand();
  }
}

void checkForConnections()
{
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central)
  {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    // Keeps track of the iteration count of spin360.
    int iteration = 0;
    
    // While central is connected
    while (central.connected())
    {
      trackRuntime();
      readCommand();
//      writeIMUDataToBLE();
//      writeTOFDataToBLE();
      writeSensorDataToBLE();
      if (pid_running)
      {
        driveWithPID();
//        driveWithKF();
      }
      else if (stunt_running)
      {
        stunt(&iteration);
      }
      else if (spin_running) {
        spin360(&iteration, initSpeed);
      }
//      delay(10);
    }

    Serial.println("Disconnected from central");
  }
}
