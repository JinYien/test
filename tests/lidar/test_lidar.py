from time import sleep

from mobile_collab_robot.misc_funcs import get_port_from_hwid
from mobile_collab_robot.obstacle_detection.lidar import Lidar

port = get_port_from_hwid("VID:PID=15D1:0000")
lidar = Lidar(port)

for i in range(50):
    a, d, t = lidar.get_scan()
    print(f"timestamp: {t}\n" f"angles: {a}\n" f"distances: {d}\n\n")
    sleep(0.01)

del lidar
