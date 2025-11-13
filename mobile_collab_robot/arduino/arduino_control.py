import logging
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports

from mobile_collab_robot.misc_funcs import get_port_from_hwid
from mobile_collab_robot.misc_funcs import setup_logger

logger = setup_logger(__name__, logging_level=logging.INFO)


class ArduinoController:
    def __init__(self, port="/dev/ttyACM0", baud=230400):  # 115200,230400
        self.sig_colors = {
            "r": 0b10,
            "y": 0b1000,
            "g": 0b1,
            "b": 0b100000,
            "o": 0b1000000,
            "p": 0b10000000,
            "n": 0b100000000,
            "c": 0b1000000000,  # light blue
            "d": 0b10000000000,  # light green
            "z": 0b100,
            "a": 255,
        }
        data_length = 600
        self.force_data = {
            x: deque(maxlen=data_length)
            for x in [
                "time",  # ★
                "Mx",  # ★
                "My",  # ★
                "Fz",  # ★
            ]
        }
        self.start_time = time.perf_counter()
        self.stop_flag = False

        self.arduino = serial.Serial(port, baud, timeout=5)
        if self.arduino is None:  # open failed
            raise "arduino open failed"

        self.arduino_thread = threading.Thread(target=self.arduino_loop)
        self.arduino_thread.start()

    def arduino_loop(self):
        time.sleep(3)
        while not self.stop_flag:
            current_t = time.perf_counter() - self.start_time
            try:
                data_str = self.arduino.readline().decode().strip()
                logger.debug(data_str)
            except Exception as e:
                continue

            try:
                split_data = data_str.split(",")  # ここや

                if len(split_data) != 3:
                    continue

                Mx, My, Fz = (int(x) for x in split_data)
                MX = Mx * 10.0 / 1023.0 - 5.0
                MY = My * 10.0 / 1023.0 - 5.0
                FZ = (Fz * 100.0 / 1023.0 - 58.0) * 1.4

                # self.force_data["time"].append(current_t)  # ★
                self.force_data["time"].append(current_t)  # ★
                self.force_data["Mx"].append(MX)  # ★
                self.force_data["My"].append(MY)  # ★
                self.force_data["Fz"].append(FZ)  # ★

                logger.debug("★", MX, MY, FZ)
                logger.debug(f"★, {MX}, {MY}, {FZ}")  # これ

            except Exception as e:
                logger.error(f"error: {e}, split_data: {split_data}")  # これ

    def set_led(self, color):
        while len(self.force_data["time"]) == 0:
            time.sleep(0.1)
        logger.info(f"set led to {color}")
        self.arduino.write(f"c{color}\n".encode())

    def set_signature(self, color):  # 1
        while len(self.force_data["time"]) == 0:  # 1
            time.sleep(0.1)  # 1
        val = int(self.sig_colors[color])  # 1
        self.arduino.write(f"s{val}\n".encode())  # 1

    def finish(self):
        self.stop_flag = True


if __name__ == "__main__":
    my_hw_ser = "8503631343035130C0B1"
    print("-")
    my_port = get_port_from_hwid(my_hw_ser)
    print("--", my_port)
    robot = ArduinoController(my_port)
    print("---", robot)
    robot.set_signature("r")
    robot.set_led("w")
