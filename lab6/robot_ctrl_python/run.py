from lab2.ble_robot.ble_python.base_ble import LOG
from lab2.ble_robot.ble_python.ble import get_ble_controller
from cmd_types import CMD

from controller import RobotControl

LOG.propagate = False

if __name__ == "__main__":
    ble = get_ble_controller()
    ble.connect()

    controller = RobotControl(ble)

    print(ble.uuid)

    ble.disconnect()
