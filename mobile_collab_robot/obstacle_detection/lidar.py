import logging
import threading
from time import sleep

import numpy as np
import serial
import serial.tools.list_ports

from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port
from mobile_collab_robot.misc_funcs import setup_logger

logger = setup_logger(__name__, logging_level=logging.INFO)


class Lidar:
    """
    Wraps hokuyo-python-lib library for ease of use
    """

    def __init__(self, port):
        self.lidar = hokuyo.Hokuyo(
            serial_port.SerialPort(serial.Serial(port=port, baudrate=115200))
        )
        self.init_lidar()

        self.scan_thread = threading.Thread(
            target=self.lidar.scanning_loop, daemon=True
        )
        self.scan_thread.start()

        self.r_scan_distance = []
        self.l_scan_distance = []

        sleep(0.5)  # wait for thread to start

    def init_lidar(self):
        res = ""
        while res != "SCIP2.0\n0Ee\n\n":
            res = self.lidar.set_scip2()

        res = ""
        while res != "RS\n00P\n\n":
            res = self.lidar.reset()

        self.lidar.enable_scanning(True)
        self.lidar.laser_off()
        self.lidar.laser_on()

    def get_scan(self):
        angles, distances, timestamp = self.lidar.get_scan()
        return angles, distances, timestamp

    def finish(self):
        self.lidar.enable_scanning(False)
        self.lidar.terminate()
        self.scan_thread.join()


def list_ports():
    for port, desc, hwid in serial.tools.list_ports.comports():
        print(f"port: {port}, desc: {desc}, hwid: {hwid}")


def get_port_from_hwid(hw_ser):
    for port, desc, hwid in serial.tools.list_ports.comports():
        if hw_ser in hwid:
            logger.info(f"Found {hw_ser} at {port}")
            return port
    else:
        logger.error(f"{hw_ser} was not found!")


def convert_raw_data(raw_angles, raw_distances, max_distance=6.0):
    """
    Convert to meter, max no detections to max distance
    """
    new_distances = np.array(raw_distances) / 1000  # convert to meter
    new_distances[new_distances < 0.008] = max_distance
    return np.array(raw_angles), new_distances


if __name__ == "__main__":
    lidar = Lidar("VID:PID=15D1:0000")

    for i in range(1):
        a, d, t = lidar.get_scan()

        print(lidar.judge_distance(a, d))
        # print(d)
        # print(f"timestamp: {t}\n" f"angles: {a}\n" f"distances: {d}\n\n")

        sleep(0.01)

    del lidar

    # def __del__(self):
    #     self.lidar.enable_scanning(False)
    #     self.lidar.terminate()
    #     self.scan_thread.join()

    # def lidar_loop(self):
    #     try:
    #         sleep(1)
    #         while True:
    #             # if self.l_on_off_flag == True:
    #                 angles, distances, timestamp = self.get_scan()
    #                 print("a",angles, distances)
    #                 point_cloud, distance_cloud = self.get_lidar_data(distances, angles)
    #                 # print("b",point_cloud)
    #                 dt = 0.1  # 時間ステップ（秒）
    #                 self.avoid_obstacles(point_cloud, distance_cloud, dt)
    #                 self.kinematics(dt)
    #                 sleep(dt)
    #     except KeyboardInterrupt:
    #         print('終了します。')
    #         print(self.laser.laser_off())

    #     finally:
    #         # ser.close()
    #         print(self.laser.laser_off())


# if __name__ == "__main__":
#     port = get_port_from_hwid("VID:PID=15D1:0000")
#     lidar = Lidar(port)

#     for i in range(50):
#         a, d, t = lidar.get_scan()
#         print(f"timestamp: {t}\n" f"angles: {a}\n" f"distances: {d}\n\n")
#         sleep(0.01)

#     del lidar
