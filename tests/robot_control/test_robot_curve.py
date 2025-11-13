from time import sleep

import numpy as np
from matplotlib import pyplot as plt

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

# move forward for 5 s then stop
robot_control.move(2, -3)
sleep(2)
robot_control.stop_free()

# move forward for 5 s then stop
robot_control.move(2, 3)
sleep(2)
robot_control.stop_free()
robot_control.release()
data = np.array(robot_control.robot_data["state"])

del robot_control

plt.plot(data[:, 0], data[:, 1])
plt.show()
