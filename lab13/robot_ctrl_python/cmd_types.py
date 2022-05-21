from enum import Enum
from pickle import STOP


class CMD(Enum):
    GET_IMU_DATA = 0
    GET_TOF_DATA = 1
    MOVE_FORWARD = 2
    MOVE_BACKWARD = 3
    MOVE_LEFT = 4
    MOVE_RIGHT = 5
    MOVE_STOP = 6
    MOVE_ASTOP = 7
    START_PID_300MM = 8
    STOP_PID_300MM = 9
    TOGGLE_DEBUG = 10
    STUNT = 11
    SPIN_360 = 12
