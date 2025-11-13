from mobile_collab_robot.misc_funcs import get_port_from_hwid
from mobile_collab_robot.robot_control import RobotController

LEFT_PORT = get_port_from_hwid("DM00KJR5")
RIGHT_PORT = get_port_from_hwid("DM00KWZP")
MIDDLE_PORT = get_port_from_hwid("DM00KV8DA")  # ★

robot_control = RobotController(
    left_port=LEFT_PORT,
    right_port=RIGHT_PORT,
    middle_port=MIDDLE_PORT,  # ★
)
robot_control.setup_middle_motor(2)

# wait for user
input()
robot_control.release()
