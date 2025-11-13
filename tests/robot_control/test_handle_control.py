from math import degrees
from time import sleep

from mobile_collab_robot.misc_funcs import get_port_from_hwid
from mobile_collab_robot.robot_control import RobotController

LEFT_PORT = get_port_from_hwid("DM00KJR5")
RIGHT_PORT = get_port_from_hwid("DM00KWZP")
MIDDLE_PORT = get_port_from_hwid("DM00KV8D")  # ★

robot_controller = RobotController(
    left_port=LEFT_PORT,
    right_port=RIGHT_PORT,
    middle_port=MIDDLE_PORT,  # ★
)
robot_controller.setup_middle_motor(2)

sleep(1)
default_velocity = 3
handle_gain = 0.5
while True:
    middle_position = degrees(robot_controller.middle_motor_data["position"][-1])
    print(middle_position)

    velocity = default_velocity
    omega = -middle_position * handle_gain
    robot_controller.move(velocity, omega)

    if -90 > middle_position > 90:
        break

robot_controller.stop_free()
robot_controller.release()
