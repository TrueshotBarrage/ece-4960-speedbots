from collections import deque
import time
from typing import NamedTuple
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from base_ble import LOG
from cmd_types import CMD

LOG.propagate = False

# UUID dict keys
tof_r_uuid_key = "RX_TOF_RIGHT"
tof_f_uuid_key = "RX_TOF_FRONT"
imu_uuid_key = "RX_IMU"
sensor_uuid_key = "RX_SENSOR"
runtime_uuid_key = "RX_BLE_RUNTIME"
speed_uuid_key = "RX_SPEED"


# NamedTuple class for sensor data. Combines TOF and IMU data
class SensorReading(NamedTuple):
    distR: float
    distF: float
    accX: float
    accY: float
    accZ: float
    gyrX: float
    gyrY: float
    gyrZ: float
    magX: float
    magY: float
    magZ: float
    temp: float


# Decodes sensor (comma-separated string) data into a SensorReading tuple
def decode_sensor_data(sensor_data: str):
    return SensorReading(*map(float, sensor_data.split(',')))


# Returns a deque of sensor reading tuples from the original unformatted data
def decode_sensor_deque(dq):
    new_deque = deque(maxlen=dq.maxlen)
    for sensor_data, time in dq:
        new_deque.append((decode_sensor_data(sensor_data), time))
    return new_deque


# Generates a plot of the sensor data numpy array
def plot_sensor_data(ndarr):
    # cols = ["time", "dist_f", "gyr_z"]
    cols = ["time", "gyr_z"]
    # df = pd.DataFrame(ndarr[:, [0, 2, 8]], columns=cols)
    df = pd.DataFrame(ndarr[:, [0, 8]], columns=cols)
    df.plot(x=cols[0], y=cols[1:], marker="o")
    plt.show()


class RobotControl():

    def __init__(self, ble, max_length=300):
        self.ble = ble
        self.max_length = max_length
        self.start_time = time.time()

        # Variables to store the latest sensor value
        self.latest_right_tof_reading = None
        self.latest_front_tof_reading = None
        self.latest_imu_reading = None
        self.latest_sensor_reading = None
        self.latest_speed_reading = None

        # Deques to store the history of all the sensor values
        # Each item in the deque is a tuple (value, time)
        self.right_tof_readings = deque(maxlen=max_length)
        self.front_tof_readings = deque(maxlen=max_length)
        self.imu_readings = deque(maxlen=max_length)
        self.sensor_readings = deque(maxlen=max_length)
        self.speed_readings = deque(maxlen=max_length)

        self.sensor_arr = None

        # Activate notifications for each characteristic
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
        self.ble.start_notify(self.ble.uuid[sensor_uuid_key],
                              self.sensor_callback_handler)
        self.ble.start_notify(self.ble.uuid[speed_uuid_key],
                              self.speed_callback_handler)
        # self.ble.start_notify(self.ble.uuid[runtime_uuid_key],
        #                       self.runtime_callback_handler)

    def start_recording(self):
        # self.right_tof_readings = deque(maxlen=self.max_length)
        # self.front_tof_readings = deque(maxlen=self.max_length)
        # self.imu_readings = deque(maxlen=self.max_length)
        self.sensor_readings = deque(maxlen=self.max_length)
        # self.speed_readings = deque(maxlen=self.max_length)

    def stop_recording(self):
        self.log_sensor_data()

    # Callback handlers for storing the history of the sensors
    def right_tof_callback_handler(self, uuid, byte_array):
        # Receive the latest right TOF reading as a byte array
        self.latest_right_tof_reading = self.ble.bytearray_to_float(byte_array)

        # Append a tuple (value, time) to the right TOF readings deque
        self.right_tof_readings.append(
            (self.latest_right_tof_reading,
             str(round(time.time() - self.start_time, 1))))

    def front_tof_callback_handler(self, uuid, byte_array):
        # Receive the latest front TOF reading as a byte array
        self.latest_front_tof_reading = self.ble.bytearray_to_float(byte_array)

        # Append a tuple (value, time) to the front TOF readings deque
        self.front_tof_readings.append(
            (self.latest_front_tof_reading,
             str(round(time.time() - self.start_time, 1))))

    def imu_callback_handler(self, uuid, byte_array):
        # Receive the latest IMU reading as a byte array
        self.latest_imu_reading = self.ble.bytearray_to_string(byte_array)

        # Append a tuple (value, time) to the IMU readings deque
        self.imu_readings.append((self.latest_imu_reading,
                                  str(round(time.time() - self.start_time, 1))))

    def sensor_callback_handler(self, uuid, byte_array):
        # Receive the latest sensor reading as a byte array
        self.latest_sensor_reading = self.ble.bytearray_to_string(byte_array)

        # Append a tuple (value, time) to the sensor readings deque
        self.sensor_readings.append((self.latest_sensor_reading,
                                     str(round(time.time() - self.start_time,
                                               3))))

    def speed_callback_handler(self, uuid, byte_array):
        # Receive the latest speed reading as a byte array
        self.latest_speed_reading = self.ble.bytearray_to_int(byte_array)

        # Append a tuple (value, time) to the speed readings deque
        self.speed_readings.append((self.latest_speed_reading,
                                    str(round(time.time() - self.start_time,
                                              1))))

    # def runtime_callback_handler(self, uuid, byte_array):
    #     self.runtime = self.ble.bytearray_to_float(byte_array)

    # Fetch the latest TOF sensor reading
    def get_tof(self):
        self.ble.send_command(CMD.GET_TOF_DATA)

    # Fetch the latest IMU reading - note the string format
    def get_imu(self):
        self.ble.send_command(CMD.GET_IMU_DATA)

    # Get the runtime of the robot
    # def get_runtime(self):
    #     self.runtime = self.ble.receive_float(self.ble.uuid[runtime_uuid_key])

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
        self.start_recording()

    # Stop PID controller for 300mm task
    def stop_pid(self):
        self.stop_recording()
        self.ble.send_command(CMD.STOP_PID_300MM)

    # Toggle debug mode
    def toggle_debug(self):
        self.ble.send_command(CMD.TOGGLE_DEBUG)

    # Log the sensor data into a numpy array
    def log_sensor_data(self):
        sensor_arr = np.zeros((len(self.sensor_readings), 13))

        for i, (reading,
                t) in enumerate(decode_sensor_deque(self.sensor_readings)):
            t = float(t)
            sensor_arr[i, 0] = t
            sensor_arr[i, 1] = reading.distR
            sensor_arr[i, 2] = reading.distF
            sensor_arr[i, 3] = reading.accX
            sensor_arr[i, 4] = reading.accY
            sensor_arr[i, 5] = reading.accZ
            sensor_arr[i, 6] = reading.gyrX
            sensor_arr[i, 7] = reading.gyrY
            sensor_arr[i, 8] = reading.gyrZ
            sensor_arr[i, 9] = reading.magX
            sensor_arr[i, 10] = reading.magY
            sensor_arr[i, 11] = reading.magZ
            sensor_arr[i, 12] = reading.temp

        self.sensor_arr = sensor_arr
        return sensor_arr

    def spin_360(self):
        self.ble.send_command(CMD.SPIN_360)
        self.start_recording()  # Must call stop_recording() after the spin stops

    def stunt(self):
        self.ble.send_command(CMD.STUNT)
