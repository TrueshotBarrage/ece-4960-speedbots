#include <ArduinoBLE.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"

//////////// BLE UUIDs ////////////
#define BLE_UUID_SERVICE "8A35A54C-6389-490E-9A34-5F6D989E157A"
#define BLE_UUID_RX_CMD "9750F60B-9C9C-4158-B620-02EC9521CD99"
#define BLE_UUID_TX_TIME "715C9B74-A522-42C6-97E1-28ECF033BC9B"

#define BLE_UUID_TX_TOF_RIGHT "2F5C6DCC-0121-4B44-90F4-4685ACB8E181"
#define BLE_UUID_TX_TOF_FRONT "07565624-13EF-4780-AFEE-BD3B38082505"
#define BLE_UUID_TX_IMU "213290BB-FC80-4269-90B7-65CE2155DEE4"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService bleService(BLE_UUID_SERVICE);
BLECStringCharacteristic cmd_string(BLE_UUID_RX_CMD, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic runtime(BLE_UUID_TX_TIME, BLERead | BLENotify);

BLEFloatCharacteristic tx_tof_r(BLE_UUID_TX_TOF_RIGHT, BLERead | BLENotify);
BLEFloatCharacteristic tx_tof_f(BLE_UUID_TX_TOF_FRONT, BLERead | BLENotify);
BLECStringCharacteristic tx_imu(BLE_UUID_TX_IMU, BLERead | BLENotify,
                                MAX_MSG_SIZE);

// RX - receive
RobotCommand robot_cmd(":|");

// TX - transmit
float ble_runtime_value = 0.0;
long interval = 100;
static long previousMillis = 0;
unsigned long currentMillis = 0;

float tx_tof_r_value = 0.0;
float tx_tof_f_value = 0.0;
EString tx_imu_value;
//////////// Global Variables ////////////

enum CommandTypes {
  MOVE_FORWARD,
};

void handle_command() {
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
  if (!success) {
    return;
  }

  // Handle the command type accordingly
  switch (cmd_type) {
    /*
     * Move the robot forward.
     */
    case MOVE_FORWARD:
      int speed;
      success = robot_cmd.get_next_value(speed);
      if (!success) return;

      Serial.print("Received speed value: ");
      Serial.println(speed);

      break;

    /*
     * The default case may not capture all types of invalid commands.
     * It is safer to validate the command string on the central device (in
     * python) before writing to the characteristic.
     */
    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}

void setup() {
  // Set up motor driver pins
  motorControlSetup();

  Serial.begin(115200);
  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Apollo BLE");
  BLE.setLocalName("Apollo BLE");
  BLE.setAdvertisedService(bleService);

  // Add BLE characteristics
  bleService.addCharacteristic(runtime);

  bleService.addCharacteristic(tx_tof_r);
  bleService.addCharacteristic(tx_tof_f);
  bleService.addCharacteristic(tx_imu);

  // Add BLE service
  BLE.addService(bleService);

  // Set initial values to prevent errors when reading for the first time on
  // central devices
  runtime.setValue(ble_runtime_value);
  tx_tof_r.writeValue(tx_tof_r_value);
  tx_tof_f.writeValue(tx_tof_f_value);

  // Clear the contents of the EString before using it
  tx_imu_value.clear();
  tx_imu_value.append("Hello world this is IMU data");

  // Write the value to the characteristic
  tx_imu.writeValue(tx_imu_value.c_str());

  // Output MAC Address
  Serial.print("[Apollo] Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();
}

void track_runtime() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    ble_runtime_value = ble_runtime_value + 0.5;
    runtime.writeValue(ble_runtime_value);

    previousMillis = currentMillis;
  }
}

void read_command() {
  // Query if the characteristic value has been written by another BLE device
  if (cmd_string.written()) {
    handle_command();
  }
}

void loop() {
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    // While central is connected
    while (central.connected()) {
      track_runtime();
      read_command();
    }

    Serial.println("Disconnected from central");
  }
}
