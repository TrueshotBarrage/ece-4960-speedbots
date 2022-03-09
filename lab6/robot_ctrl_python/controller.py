from collections import deque
import time

from lab2.ble_robot.ble_python.ble import get_ble_controller
from lab2.ble_robot.ble_python.base_ble import LOG
from cmd_types import CMD

LOG.propagate = False

# UUID dict keys
tof_r = "RX_TOF_RIGHT"
tof_f = "RX_TOF_FRONT"
imu = "RX_IMU"


class RobotControl():
    # Initialize Function
    def __init__(self, ble):
        self.ble = ble

        # Variables to store the latest sensor value
        self.latest_tof_right_reading = None
        self.latest_tof_front_reading = None
        self.latest_imu_reading = None

        # Deques to store the history of all the sensor values
        # Each item in the deque is a tuple (value, time)
        self.right_tof_readings = deque(maxlen=100)
        self.front_tof_readings = deque(maxlen=100)
        self.imu_readings = deque(maxlen=100)

        # Activate notifications (if required)
        self.setup_notify()

    # A function to activate various notifications (if required)
    def setup_notify(self):
        self.ble.start_notify(self.ble.uuid[tof_r],
                              self.right_tof_callback_handler)
        self.ble.start_notify(self.ble.uuid[tof_f],
                              self.front_tof_callback_handler)
        self.ble.start_notify(self.ble.uuid[imu], self.imu_callback_handler)

    # Callback handlers for storing the history of the sensors
    def right_tof_callback_handler(self, uuid, byte_array):
        # Append a tuple (value, time) to the right tof readings deque
        self.right_tof_readings.append(
            (self.ble.bytearray_to_float(byte_array), time.time()))

    def front_tof_callback_handler(self, uuid, byte_array):
        # Append a tuple (value, time) to the front tof readings deque
        self.front_tof_readings.append(
            (self.ble.bytearray_to_float(byte_array), time.time()))

    def imu_callback_handler(self, uuid, byte_array):
        # Append a tuple (value, time) to the imu readings deque
        self.imu_readings.append(
            (self.ble.bytearray_to_float(byte_array), time.time()))

    # Fetch the right TOF sensor reading
    def get_right_tof(self):
        self.latest_tof_right_reading = self.ble.receive_float(
            self.ble.uuid[tof_r])

    # Fetch the front TOF sensor reading
    def get_front_tof(self):
        self.latest_tof_front_reading = self.ble.receive_float(
            self.ble.uuid[tof_f])

    # Fetch the latest IMU reading - note the string format
    def get_imu(self):
        self.latest_imu_reading = self.ble.receive_string(self.ble.uuid[imu])

    # Instruct the robot to move forward
    def move_forward(self, speed):
        raise NotImplementedError()
        ble.send_command(CMD.MOVE_FORWARD, speed)

    # Stop robot motion
    def stop(self):
        raise NotImplementedError()