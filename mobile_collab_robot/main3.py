import threading
import time
import logging
import numpy as np
import sounddevice as sd
import math

from mobile_collab_robot.arduino import ArduinoController
from mobile_collab_robot.misc_funcs import get_port_from_hwid
# from mobile_collab_robot.obstacle_detection import ObstacleDetection
from mobile_collab_robot.obstacle_detection.judge_pathway import JudgePathway
from mobile_collab_robot.obstacle_avoidance import ObstacleAvoidance
# from mobile_collab_robot.obstacle_detection.obstacle import Obstacle
from mobile_collab_robot.robot_control import RobotController
from obstacle_detection.lidar import Lidar
# from mobile_collab_robot.obstacle_detection.lidar import Lidar

# sd.default.latency = "low"
LEFT_PORT = get_port_from_hwid("DM00KJR5")
RIGHT_PORT = get_port_from_hwid("DM00KWZP")
# MIDDLE_PORT = get_port_from_hwid("DM00KVVM")  # ★DM00KV8D
MIDDLE_PORT = get_port_from_hwid("DM00KV8D")  # ★DM00KV8D
ARDUINO_PORT = get_port_from_hwid("8503631343035130C0B1")  #
Lidar_PORT = get_port_from_hwid("15D1:0000")  #

#力センサ(onーoff)なしver
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
        # self.obstacle_detection = ObstacleDetection(port=lidar_port)
        self.judge_pathway = JudgePathway(port=lidar_port)
        # self.obstacle = Obstacle()

        self.detection = ObstacleAvoidance(angles=None,distances = None,barrier_distance=5,max_range=3,min_range=0,)

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
        self.corner_flag = False

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
        angles = self.judge_pathway.use_angles
        distances = self.judge_pathway.use_distances
        l_angles = angles.tolist()
        l_distances = distances.tolist()
        # print(l_distances)
        region = self.detection.get_detection_region()
        x_min, x_max = self.detection.get_min_width()
        self.detection.update_data(
            angles,
            distances,
        )
        middle_position = -math.degrees(self.motor_controller.middle_motor_data["position"][-1])




        if self.debug_mode:
            print("@@@")
            return

        try:
            # print("####",self.judge_pathway.decide_pathway)
            # print("$$$$",self.judge_pathway.near_segments)
            # print("middle", self.motor_controller.middle_motor_data["position"][-1])
            # print("1") 
            # 
            # time.sleep(5)  



            # 暴走チェック
            if (
                    self.motor_controller.left_measurements["velocity"] > 14
                    or self.motor_controller.left_measurements["velocity"] < -14
                    or self.motor_controller.right_measurements["velocity"] > 14
                    or self.motor_controller.right_measurements["velocity"] < -14
            ) or (
                    abs(
                        self.motor_controller.left_measurements["velocity"]
                        + self.motor_controller.right_measurements["velocity"]
                    )
                    >= 14
            ):
                print("#####################################################")
                self.motor_controller.stop_free()
                self.arduino_controller.set_led("p")
                time.sleep(2)
                return



            # # on-off check
            # # print(self.arduino_controller.force_data["Mx"][-1])
            # # self.on_off_flag = True
            # if self.arduino_controller.force_data["Mx"][-1] < -0.4:
            #     # print("aaaaaa")
            #     if self.on_off_flag:
            #         self.on_off_flag = False
            #         # print("off")
            #         self.motor_controller.stop()
            #         self.arduino_controller.set_led("r")
            #         # self.lidar.laser.laser_off()
            #         # self.lidar.l_on_off_flag = False
            #         time.sleep(2)
            #         return
            #     else:
            #         self.on_off_flag = True
            #         # print("on")
            #         self.arduino_controller.set_led("g")
            #         # if self.lidar.l_on_off_flag == False:
            #         #     # self.lidar.laser.laser_on()
            #         #     # self.lidar.l_on_off_flag = True
            #         time.sleep(2)
            #         return

            # if not self.on_off_flag:
            #     # print("^^^^^^^^^^^^^^^^^^^")
            #     return

            #todo:曲がり角判定
            # if self.avoid_flag:
                
            #     self.avoid_flag = False
            #     return
            r_front_distances,r_rear_distances,l_front_distances,l_rear_distances = self.judge_pathway.get_side_data(l_angles,l_distances,max_angle = 100,min_angle= 80,border = 90)
            if self.judge_pathway.judge_corner(r_front_distances,r_rear_distances,barrier_distance = 1,over_distance = 1.5): #barrier_distance = 1.5,over_distance = 2.0
                self.arduino_controller.set_led("p")
                time.sleep(1)
                self.motor_controller.middle_stop()

                print("------r_corner---------")
                self.motor_controller.stop()
                self.corner_flag = True
                self.on_off_flag = False
                time.sleep(3)#add
                return

            if self.judge_pathway.judge_corner(l_front_distances,l_rear_distances,barrier_distance = 1,over_distance = 1.5):
                self.arduino_controller.set_led("d")
                time.sleep(1)
                self.motor_controller.middle_stop()
                print("******l_corner********")
                self.motor_controller.stop()
                self.corner_flag = True
                self.on_off_flag = False
                time.sleep(3)#add
                return         
     


  
            #todo:障害物回避 また今度
            # direction = self.detection.get_optimal_pathway()
            # print("~~~~~~~~~",direction)

            # if direction != 0:
            #     self.avoid_flag = True
                          
            #     # range_angles,range_distances,count = self.judge_pathway.get_range_segments(l_angles, l_distances,middle_position, barrier_distance=1.5)
           
            #     # while(count >= 10):
            #     #     self.arduino_controller.set_led("b")
            #     #     self.motor_controller.middle_stop()
            #     #     # self.motor_controller.right_motor_hold()
            #     #     # self.motor_controller.leht_motor_hold()
            #     #     self.motor_controller.stop()
            #     #     angles = self.judge_pathway.use_angles
            #     #     distances = self.judge_pathway.use_distances
            #     #     l_angles = angles.tolist()
            #     #     l_distances = distances.tolist()
            #     #     middle_position = -math.degrees(self.motor_controller.middle_motor_data["position"][-1])
            #     #     range_angles,range_distances,count = self.judge_pathway.get_range_segments(l_angles, l_distances,middle_position, barrier_distance=2)
            #     #     print("!",count, middle_position, len(range_angles))

            #     self.arduino_controller.set_led("o")
            #     self.motor_controller.move_by_lidar(
            #     -direction
            # )
            #     return
            # if self.avoid_flag:
            #     time.sleep(0.5)
            #     self.avoid_flag = False

            #todo:壁の当たり判定

            # middle_position = -math.degrees(self.motor_controller.middle_motor_data["position"][-1])
            range_angles,range_distances,count = self.judge_pathway.get_range_segments(l_angles, l_distances,middle_position, barrier_distance=1)
            # print(count,range_angles, range_distances,len(range_angles),len(range_distances))
            # print(count)

           
            while(count >= 10):
                self.arduino_controller.set_led("b")
                self.motor_controller.middle_stop()
                # self.motor_controller.right_motor_hold()
                # self.motor_controller.leht_motor_hold()
                self.motor_controller.stop()
                angles = self.judge_pathway.use_angles
                distances = self.judge_pathway.use_distances
                l_angles = angles.tolist()
                l_distances = distances.tolist()
                middle_position = -math.degrees(self.motor_controller.middle_motor_data["position"][-1])
                range_angles,range_distances,count = self.judge_pathway.get_range_segments(l_angles, l_distances,middle_position, barrier_distance=2)
                print("!",count, middle_position, len(range_angles))

            # #todo:曲がり角判定
            # r_front_distances,r_rear_distances,l_front_distances,l_rear_distances = self.judge_pathway.get_side_data(l_angles,l_distances,max_angle = 100,min_angle= 80,border = 90)
            # if self.judge_pathway.judge_corner(r_front_distances,r_rear_distances,barrier_distance = 1.5,over_distance = 2.0):
            #     self.arduino_controller.set_led("p")
            #     time.sleep(1.5)
            #     self.motor_controller.middle_stop()

            #     print("------r_corner---------")
            #     self.motor_controller.stop()
            #     self.corner_flag = True
            #     self.on_off_flag = False
            #     return

            # if self.judge_pathway.judge_corner(l_front_distances,l_rear_distances,barrier_distance = 1.5,over_distance = 2.0):
            #     self.arduino_controller.set_led("d")
            #     time.sleep(1.5)
            #     self.motor_controller.middle_stop()
            #     print("******l_corner********")
            #     self.motor_controller.stop()
            #     self.corner_flag = True
            #     self.on_off_flag = False
            #     return
           

            # 物体認識されてない，★力センサによる速度制御
            # 物体認識時，対象物が遠いとき，★力センサによる速度制御
            # if (
            #         self.arduino_controller.force_data["object_x"][-1] is np.nan
            #         or self.arduino_controller.force_data["distance_to_object"][-1] > 160
            # ):
            self.arduino_controller.set_led("c")
            print("m")
            self.motor_controller.basic_move(
                current_t,
                # self.arduino_controller.force_data["Fz"][-1],
                self.motor_controller.middle_motor_data["position"][-1],
                # self.arduino_controller.data["distance_from_center"][-1],
            )
            return
        


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