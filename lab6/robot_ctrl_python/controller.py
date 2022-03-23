from collections import deque
import time

from ble import get_ble_controller
from base_ble import LOG
from cmd_types import CMD

LOG.propagate = False

# UUID dict keys
tof_r_uuid_key = "RX_TOF_RIGHT"
tof_f_uuid_key = "RX_TOF_FRONT"
imu_uuid_key = "RX_IMU"
runtime_uuid_key = "RX_BLE_RUNTIME"


class RobotControl():
    # Initialize Function
    def __init__(self, ble):
        self.ble = ble
        self.runtime = 0.0

        # Variables to store the latest sensor value
        self.latest_right_tof_reading = None
        self.latest_front_tof_reading = None
        self.latest_imu_reading = None

        # Deques to store the history of all the sensor values
        # Each item in the deque is a tuple (value, time)
        self.right_tof_readings = deque(maxlen=100)
        self.front_tof_readings = deque(maxlen=100)
        self.imu_readings = deque(maxlen=100)

        # Activate notifications (if required)
        self.setup_notify()

    # A function to activate notifications for the sensor readings.
    # Note that the readings are updated by default, but can also be manually
    # read by calling the get_tof() and get_imu() functions.
    def setup_notify(self):
        self.ble.start_notify(self.ble.uuid[tof_r_uuid_key],
                              self.right_tof_callback_handler)
        self.ble.start_notify(self.ble.uuid[tof_f_uuid_key],
                              self.front_tof_callback_handler)
        self.ble.start_notify(self.ble.uuid[imu_uuid_key],
                              self.imu_callback_handler)

    # Callback handlers for storing the history of the sensors
    def right_tof_callback_handler(self, uuid, byte_array):
        # Receive the latest right TOF reading as a byte array
        self.latest_right_tof_reading = self.ble.bytearray_to_float(byte_array)

        # Append a tuple (value, time) to the right TOF readings deque
        self.right_tof_readings.append(
            (self.latest_right_tof_reading, time.time()))

    def front_tof_callback_handler(self, uuid, byte_array):
        # Receive the latest front TOF reading as a byte array
        self.latest_front_tof_reading = self.ble.bytearray_to_float(byte_array)

        # Append a tuple (value, time) to the front TOF readings deque
        self.front_tof_readings.append(
            (self.latest_front_tof_reading, time.time()))

    def imu_callback_handler(self, uuid, byte_array):
        # Receive the latest IMU reading as a byte array
        self.latest_imu_reading = self.ble.bytearray_to_string(byte_array)

        # Append a tuple (value, time) to the IMU readings deque
        self.imu_readings.append((self.latest_imu_reading, time.time()))

    # Fetch the latest TOF sensor reading
    def get_tof(self):
        self.ble.send_command(CMD.GET_TOF_DATA)

    # Fetch the latest IMU reading - note the string format
    def get_imu(self):
        self.ble.send_command(CMD.GET_IMU_DATA)

    # Get the runtime of the robot
    def get_runtime(self):
        self.runtime = self.ble.receive_float(self.ble.uuid[runtime_uuid_key])

    # Instruct the robot to move forward
    def move_forward(self, speed):
        self.ble.send_command(CMD.MOVE_FORWARD, speed)

    # Instruct the robot to move backward
    def move_backward(self, speed):
        self.ble.send_command(CMD.MOVE_BACKWARD, speed)

    # Instruct the robot to rotate left (counterclockwise)
    def rotate_left(self, speed):
        self.ble.send_command(CMD.MOVE_LEFT, speed)

    # Instruct the robot to rotate right (clockwise)
    def rotate_right(self, speed):
        self.ble.send_command(CMD.MOVE_RIGHT, speed)

    # Stop robot motion
    def stop(self):
        self.ble.send_command(CMD.MOVE_STOP)

    # Start PID controller for 300mm task
    def start_pid(self):
        self.ble.send_command(CMD.START_PID_300MM)

    # Stop PID controller for 300mm task
    def stop_pid(self):
        self.ble.send_command(CMD.STOP_PID_300MM)

    # Toggle debug mode
    def toggle_debug(self):
        self.ble.send_command(CMD.TOGGLE_DEBUG)
