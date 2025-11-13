import threading
import time
import logging
import numpy as np
import sounddevice as sd

from mobile_collab_robot.arduino import ArduinoController
from mobile_collab_robot.misc_funcs import get_port_from_hwid
from mobile_collab_robot.obstacle_detection import ObstacleDetection
# from mobile_collab_robot.obstacle_detection.judge_pathway import JudgePathway
from mobile_collab_robot.robot_control import RobotController
# from mobile_collab_robot.obstacle_detection.lidar import Lidar

# sd.default.latency = "low"
LEFT_PORT = get_port_from_hwid("DM00KJR5")
RIGHT_PORT = get_port_from_hwid("DM00KWZP")
MIDDLE_PORT = get_port_from_hwid("DM00KVVM")  # ★
ARDUINO_PORT = get_port_from_hwid("8503631343035130C0B1")  #
Lidar_PORT = get_port_from_hwid("15D1:0000")  #


class MobileRobot:
    def __init__(
            self,
            left_port=LEFT_PORT,
            right_port=RIGHT_PORT,
            middle_port=MIDDLE_PORT,  # ★
            arduino_port=ARDUINO_PORT,
            lidar_port=Lidar_PORT
    ):
        self.arduino_controller = ArduinoController(arduino_port)
        self.motor_controller = RobotController(
            left_port=left_port, right_port=right_port, middle_port=middle_port
        )
        self.obstacle_detection = ObstacleDetection(port=lidar_port)
        self.obstacle_pathway = JudgePathway(port=lidar_port)

        self.arduino_controller.set_signature("z")
        self.arduino_controller.set_led("r")

        # self.target_signatures = [1, 4, 2, 6]
        self.target_signatures = [
            "r",
            "g",
            "b",
            "r",
            "r",
            "r",
        ]  # Red, Yellow, Green, blue
        self.current_target_index = 0
        self.reference_distance = 0
        self.stop_distance = 40

        self.sleep_start_time = None
        self.on_off_flag = False
        self.avoid_flag = False

        self.tracking_target_tone = self.make_tone(600, 0.5)
        self.target_reached_tone = self.make_tone(800, 0.5)
        self.exchange_tone = self.make_tone(800, 2)
        self.start_time = time.time()

        # self.control_timer = RepeatTimer(0.1, self.control_loop)
        self.control_timer = RepeatTimer(0.01, self.control_loop)
        self.control_timer.start()
        self.arduino_controller.set_signature(
            self.target_signatures[self.current_target_index]  # 1
            # "r","g"
        )

        self.debug_mode = False

    @staticmethod
    def make_tone(freq_hz, duration_s, sps=44100):
        each_sample_number = np.arange(duration_s * sps)
        waveform = np.sin(2 * np.pi * each_sample_number * freq_hz / sps)
        return waveform

    def control_loop(self):
        current_t = time.time() - self.start_time
        if self.debug_mode:
            return

        try:
            print("1")
            # 暴走チェック
            if (
                    self.motor_controller.left_measurements["velocity"] > 13
                    or self.motor_controller.left_measurements["velocity"] < -13
                    or self.motor_controller.right_measurements["velocity"] > 13
                    or self.motor_controller.right_measurements["velocity"] < -13
            ) or (
                    abs(
                        self.motor_controller.left_measurements["velocity"]
                        + self.motor_controller.right_measurements["velocity"]
                    )
                    >= 9
            ):
                self.motor_controller.stop_free()
                self.arduino_controller.set_led("p")
                time.sleep(5)
                return
            print("2")
            # on-off check
            if self.arduino_controller.force_data["Mx"][-1] < -0.4:
                if self.on_off_flag:
                    self.on_off_flag = False
                    # print("off")
                    self.motor_controller.stop()
                    self.arduino_controller.set_led("r")
                    # self.lidar.laser.laser_off()
                    # self.lidar.l_on_off_flag = False
                    time.sleep(3)
                    return
                else:
                    self.on_off_flag = True
                    # print("on")
                    self.arduino_controller.set_led("g")
                    # if self.lidar.l_on_off_flag == False:
                    #     # self.lidar.laser.laser_on()
                    #     # self.lidar.l_on_off_flag = True
                    time.sleep(3)
                    return

            if not self.on_off_flag:
                return


            print("####",self.obstacle_pathway.judged_pathway)
            obs = self.obstacle_detection.judge
            print(obs)
            while obs:
                print("obstacle")
                self.arduino_controller.set_led("b")
                self.motor_controller.middle_motor_hold()
                self.motor_controller.stop()
                time.sleep(3)
                self.motor_controller.stop_free()
                # self.motor_controller.stop_free()
                sd.play(self.exchange_tone)
                obs = self.obstacle_detection.judge
                # time.sleep(5)

            wall = self.obstacle_detection.Wall
            dire = self.obstacle_detection.Dire
            print("★",wall)

            if wall == True:
                print("wall")
                # self.motor_controller.stop_free()
                self.motor_controller.stop()
                self.motor_controller.middle_stop()
                self.on_off_flag = False
                if dire < 0:
                    print("----------------------------------left")
                    self.arduino_controller.set_led("p")
                    time.sleep(2)
                else:
                    print("------------------------------------right")
                    self.arduino_controller.set_led("n")
                    time.sleep(2)
                return
                

            # if self.motor_controller.user_pulling(
            #         current_t,
            #         self.arduino_controller.force_data["Fz"][-1],
            # ):
            #     print("user pulled")
            #     self.arduino_controller.set_led("o")
            #     self.motor_controller.stop()
            #     self.motor_controller.middle_motor_hold()
            #     self.motor_controller.stop_free()
            #     sd.play(self.exchange_tone)
            #     time.sleep(5)
            #     return

            # if (
            #     self.lidar.m_flag                
            # ):
            #     self.arduino_controller.set_led("z")
            #     self.motor_controller.middle_motor_hold()
            #     self.motor_controller.stop_free()

            # 物体認識されてない，★力センサによる速度制御
            # 物体認識時，対象物が遠いとき，★力センサによる速度制御
            # if (
            #         self.arduino_controller.force_data["object_x"][-1] is np.nan
            #         or self.arduino_controller.force_data["distance_to_object"][-1] > 160
            # ):
            self.arduino_controller.set_led("c")
            print("m")
            self.motor_controller.move_by_force(
                current_t,
                self.arduino_controller.force_data["Fz"][-1],
                self.motor_controller.middle_motor_data["position"][-1],
                # self.arduino_controller.data["distance_from_center"][-1],
            )
            return
        

            # self.arduino_controller.set_led("y")
            # # self.motor_controller.middle_motor()
            # self.motor_controller.move_avoid_target(
            #     current_t,
            #     self.arduino_controller.data["distance_to_object"][-1],
            #     self.arduino_controller.data["distance_from_center"][-1],
            #     self.arduino_controller.data["Fz"][-1],
            # )

            # if self.arduino_controller.target_reached(
            #         current_t, self.stop_distance
            # ):  # complex エラー
            #     print("target reached")
            #     self.arduino_controller.set_led("w")
            #     print("move to next color")
            #     self.current_target_index += 1
            #     if self.current_target_index >= len(self.target_signatures):
            #         self.current_target_index = 0
            #     self.arduino_controller.set_signature(
            #         self.target_signatures[self.current_target_index]
            #     )
            #     time.sleep(1)
            #     return
            # if (
            #         self.lidar.m_flag == True
            # ):
            #     self.arduino_controller.set_led("z")
            #     self.motor_controller.middle_motor_hold()
            #     self.motor_controller.stop_free()





        # TODO; middleモータの制御， self.arduino_controller.data["Mx"][-1]から最新の力センサの値を取得して self.motor_control.handle_target()を組む
        except KeyboardInterrupt:  # memo; 後で考える
            for h in range(10):
                print("interrupted!")


class RepeatTimer(threading.Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    robot = MobileRobot()
    # robot = MobileRobot(
    #     left_port="/dev/ttyUSB0",
    #     right_port="/dev/ttyUSB1",
    #     middle_port="/dev/ttyUSB2",  # ★
    #     arduino_port="/dev/ttyACM0",
    # )

    input()
    del robot