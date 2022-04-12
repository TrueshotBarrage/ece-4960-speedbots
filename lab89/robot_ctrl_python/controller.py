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
runtime_uuid_key = "RX_BLE_RUNTIME"
speed_uuid_key = "RX_SPEED"


class IMUSensorReading(NamedTuple):
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


def decode_imu_data(imu_data: str):
    return IMUSensorReading(*map(float, imu_data.split(',')))


# Returns a deque of IMU sensor reading tuples from the original unformatted data
def decode_imu_deque(dq):
    new_deque = deque(maxlen=dq.maxlen)
    for imu_data, time in dq:
        new_deque.append((decode_imu_data(imu_data), time))
    return new_deque


def plot_time_series_data(ndarr):
    cols = [
        "time", "speed", "dist_r", "dist_f", "acc_x", "acc_y", "acc_z", "gyr_x",
        "gyr_y", "gyr_z", "mag_x", "mag_y", "mag_z", "temp"
    ]
    df = pd.DataFrame(ndarr, columns=cols)
    df.plot(x=cols[0], y=cols[1:], marker="o")  #.interpolate(method="linear")
    plt.show()


class RobotControl():
    # Initialize Function
    def __init__(self, ble, max_length=300):
        self.ble = ble
        self.max_length = max_length
        self.start_time = time.time()

        # Variables to store the latest sensor value
        self.latest_right_tof_reading = None
        self.latest_front_tof_reading = None
        self.latest_imu_reading = None
        self.latest_speed_reading = None

        # Deques to store the history of all the sensor values
        # Each item in the deque is a tuple (value, time)
        self.right_tof_readings = deque(maxlen=max_length)
        self.front_tof_readings = deque(maxlen=max_length)
        self.imu_readings = deque(maxlen=max_length)
        self.speed_readings = deque(maxlen=max_length)

        # self.sensor_dict = {}
        self.sensor_arr = None

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
        self.ble.start_notify(self.ble.uuid[speed_uuid_key],
                              self.speed_callback_handler)
        # self.ble.start_notify(self.ble.uuid[runtime_uuid_key],
        #                       self.runtime_callback_handler)

    def start_recording(self):
        self.right_tof_readings = self.front_tof_readings = \
        self.imu_readings = self.speed_readings = deque(maxlen=self.max_length)

    def stop_recording(self):
        self.log_data()

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

    def log_data(self):
        # First, generate a set of all the times
        time_set = set()
        combined_sensor_readings = [
            self.speed_readings, self.right_tof_readings,
            self.front_tof_readings,
            decode_imu_deque(self.imu_readings)
        ]
        for sensor in combined_sensor_readings:
            for reading in sensor:
                time_set.add(float(reading[1]))

        # Then, sort the set and convert it to an array and a mapping to indices
        time_arr = np.array(sorted(time_set))
        t_to_i = {time: i for i, time in enumerate(time_arr)}

        # Finally, generate a list of sensor readings for each time
        sensor_arr = np.full((len(time_arr), 14), np.nan)
        sensor_arr[:, 0] = time_arr

        for c, sensor in enumerate(combined_sensor_readings):
            for reading, t in sensor:
                t = float(t)
                if type(reading) == IMUSensorReading:
                    sensor_arr[t_to_i[t], c + 1] = reading.accX
                    sensor_arr[t_to_i[t], c + 2] = reading.accY
                    sensor_arr[t_to_i[t], c + 3] = reading.accZ
                    sensor_arr[t_to_i[t], c + 4] = reading.gyrX
                    sensor_arr[t_to_i[t], c + 5] = reading.gyrY
                    sensor_arr[t_to_i[t], c + 6] = reading.gyrZ
                    sensor_arr[t_to_i[t], c + 7] = reading.magX
                    sensor_arr[t_to_i[t], c + 8] = reading.magY
                    sensor_arr[t_to_i[t], c + 9] = reading.magZ
                    sensor_arr[t_to_i[t], c + 10] = reading.temp
                else:
                    sensor_arr[t_to_i[t], c + 1] = reading

        self.sensor_arr = sensor_arr
        return sensor_arr

    # Requires testing by hand to adjust values
    def spin_360(self):
        for i in range(14):
            self.rotate_right(100)
            time.sleep(0.5)
            self.stop()
            time.sleep(0.5)
