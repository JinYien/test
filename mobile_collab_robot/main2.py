import logging
import math
import threading
import time

import numpy as np
import sounddevice as sd

from mobile_collab_robot.arduino import ArduinoController
from mobile_collab_robot.misc_funcs import (
    get_port_from_hwid,
    setup_logger,
    setup_realtime_filter,
    realtime_filter,
)
from mobile_collab_robot.obstacle_detection import (
    Lidar,
    ObstacleAvoidance,
    TurningPoint,
    FrontObstacle,
    convert_raw_data,
)
from mobile_collab_robot.robot_control import RobotController

logger = setup_logger(__name__, logging_level=logging.INFO)

# LEFT_PORT = get_port_from_hwid("DM00KJR5")
# RIGHT_PORT = get_port_from_hwid("DM00KWZP")
RIGHT_PORT = get_port_from_hwid("DM00KJR5")
LEFT_PORT = get_port_from_hwid("DM00KWZP")
MIDDLE_PORT = get_port_from_hwid("DM00KV8D")  # ★DM00KV8D
ARDUINO_PORT = get_port_from_hwid("8503631343035130C0B1")  #
LIDAR_PORT = get_port_from_hwid("15D1:0000")  #

sd.default.device = 11


class ControlCenter:
    def __init__(
        self,
        left_port=LEFT_PORT,
        right_port=RIGHT_PORT,
        middle_port=MIDDLE_PORT,  # ★
        arduino_port=ARDUINO_PORT,
        lidar_port=LIDAR_PORT,
    ):
        #デフォルトスピード
        self.default_velocity = 0.55#0.4
        #比例ゲイン
        self.handle_gain1 = 0.08 #ハンドル角度
        # self.handle_gain = 0.08
        self.distance_gain = 0.5  # 0.40,0.6 #障害物との距離
        self.angle_gain = 0.06  # 0.04 #障害物の角度
        self.distance_model = "exponential"

        self.b_i, self.a_i, self.z_i = setup_realtime_filter(6, 30)

        self.arduino_controller = ArduinoController(arduino_port)
        self.robot_controller = RobotController(
            left_port=left_port, right_port=right_port, middle_port=middle_port
        )
        self.robot_controller.setup_middle_motor(0.1)
        self.robot_controller.set_max_torque(0.3)

        self.lidar = Lidar(lidar_port)
        self.angles, self.distances = np.array([]), np.array([])
        self.obstacle_position, self.avoidance_vector = (0, 0), 0
        self.avoidance_vector_buffer = np.zeros(500)

        self.turning_point = TurningPoint(
            angles=None,
            distances=None,
            min_distance_to_wall=0.8,#0.8
            min_distance_to_opening=1,#1
        )
        self.obstacle_avoidance = ObstacleAvoidance(
            min_width=0.4,  # 0.6
            angle_range=88,  # 3
            barrier_distance=1.1,  # 1.5,1.3,1
        )
        self.get_front_obstacle = FrontObstacle(
            angles=None,
            distances=None,
            position=None,
            min_distance_to_obstacle=0.4,#0.6
        )

        self.arduino_controller.set_led("r")

        self.sleep_start_time = None
        self.on_flag = False
        self.stopped_flag = False
        self.f_stopped_flag = False
        self.turning_point_flag = False
        self.no_pathway_flag = False
        self.f_obstacle_flag = False

        self.sound_fs = 48000
        sd.default.samplerate = self.sound_fs

        # 音
        self.turning_point_tone = self.make_tone(900, 1)
        self.no_pathway_tone = self.make_tone(600, 1)
        self.exchange_tone = self.make_tone(300, 1)

        sd.play(self.exchange_tone)

        self.start_time = time.time()

        self.control_timer = RepeatTimer(0.01, self.control_loop)
        self.control_timer.start()

    def make_tone(self, freq_hz, duration_s):
        sps = self.sound_fs
        each_sample_number = np.arange(duration_s * sps)
        waveform = np.sin(2 * np.pi * each_sample_number * freq_hz / sps)
        return waveform

    def control_loop(self):
        current_t = time.time() - self.start_time

        angles, distances, timestamp = self.lidar.get_scan()
        self.angles, self.distances = convert_raw_data(
            angles, distances, max_distance=6
        )  # ★★★★

        if self.arduino_controller.force_data["Mx"][-1] < -0.5:
            if self.on_flag:
                logger.info("Turn off!")
                self.robot_controller.stop()
                self.on_flag = False
                sd.play(self.exchange_tone)
                self.arduino_controller.set_led("r")
                time.sleep(1.5)
                return
            else:
                logger.info("Turn on!")
                self.on_flag = True
                self.arduino_controller.set_led("g")
                time.sleep(1.5)
                return

        if not self.on_flag:
            self.robot_controller.stop()
            return

        # if not self.on_flag:
        #     self.robot_controller.stop()
        #     if self.turning_point_flag:
        #         print("turn")
        #         sd.play(self.turning_point_tone)  # 音
        #         time.sleep(1.0)
        #     if self.no_pathway_flag:
        #         print("nopathway")
        #         sd.play(self.no_pathway_tone)  # 音
        #         time.sleep(1.0)

        #     return

        if self.stopped_flag:
            if self.arduino_controller.force_data["Fz"][-1] < -7.5:#8
                self.stopped_flag = False
                self.f_stopped_flag = False
                self.turning_point_flag = False
                self.no_pathway_flag = False
                self.f_obstacle_flag = False
                self.arduino_controller.set_led("g")
            if self.no_pathway_flag or self.f_stopped_flag or self.f_obstacle_flag:
                if self.arduino_controller.force_data["Fz"][-1] > 4:#6
                    print("!!!!!back")
                    velocity = self.default_velocity
                    middle_position = math.degrees(
                        self.robot_controller.middle_motor_data["position"][-1]
                    )
                    self.arduino_controller.set_led("c")
                    handle_gain = self.angle_gain
                    omega = middle_position * handle_gain / 3
                    self.robot_controller.move(-velocity / 2, omega)
                    # time.sleep(1)
                    return
                else:
                    self.arduino_controller.set_led("p")

            self.robot_controller.stop()
            return

        if not self.f_stopped_flag:
            if self.arduino_controller.force_data["Fz"][-1] > 20:
                print("stop")
                self.stopped_flag = True
                self.f_stopped_flag = True
                self.arduino_controller.set_led("o")
                self.robot_controller.stop()
                return

        handle_gain = self.handle_gain1
        self.obstacle_avoidance.update_data(
            self.angles,
            self.distances,
        )
        self.turning_point.update_data(
            self.angles,
            self.distances,
        )
        self.get_front_obstacle.update_data(
            self.angles,
            self.distances,
            math.degrees(self.robot_controller.middle_motor_data["position"][-1]),
        )
        optimal_pathway = self.obstacle_avoidance.get_optimal_pathway()
        turning_point = self.turning_point.run()
        logger.info(optimal_pathway)
        front_obstacle = self.get_front_obstacle.run()

        if front_obstacle:
            self.robot_controller.stop()
            self.f_obstacle_flag = True
            self.stopped_flag = True
            self.arduino_controller.set_led("b")
            # sd.play(self.no_pathway_tone)
            return

        if turning_point:
            self.robot_controller.stop()
            self.stopped_flag = True
            self.turning_point_flag = True
            sd.play(self.turning_point_tone)  # 音
            self.arduino_controller.set_led("y")
            return

        if optimal_pathway == "No pathways":
            print("!", optimal_pathway)
            self.robot_controller.stop()
            self.stopped_flag = True
            self.no_pathway_flag = True
            sd.play(self.no_pathway_tone)
            self.arduino_controller.set_led("p")
            return
        else:
            self.arduino_controller.set_led("g")
            if optimal_pathway == "No obstacles":
                angle, distance, clockwise = 0, 0, True
                self.obstacle_position = (0, 0)
                self.avoidance_vector = 0
                handle_gain = self.handle_gain1
            else:
                angle, distance, clockwise = optimal_pathway
                direction = 1 if clockwise else -1
                self.obstacle_position = self.obstacle_avoidance.polar_to_cartesian(
                    angle, distance
                )
                print("pos", self.obstacle_position, self.obstacle_position[1])
                angle_vector = self.angle_gain * abs(angle)

                # angle_vector = self.angle_gain * abs(angle)
                if self.distance_model == "linear":
                    distance_vector = self.distance_gain * (
                        self.obstacle_avoidance.barrier_distance - distance
                    )
                elif self.distance_model == "exponential":
                    distance_vector = (
                        distance / self.obstacle_avoidance.barrier_distance
                    ) ** -self.distance_gain - 1  # -1
                else:
                    distance_vector = 0
                if distance <= self.obstacle_avoidance.barrier_distance:
                    handle_gain = (
                        distance * self.handle_gain1
                    ) / self.obstacle_avoidance.barrier_distance

                self.avoidance_vector = (angle_vector + distance_vector) * direction
                # handle_gain = self.handle_gain2

        # # filter avoidance vector
        # self.avoidance_vector_buffer[:-1] = self.avoidance_vector_buffer[1:]
        # self.avoidance_vector_buffer[-1] = self.avoidance_vector

        # self.avoidance_vector_buffer, self.z_i = realtime_filter(
        #     self.avoidance_vector_buffer, self.b_i, self.z_i, self.a_i
        # )
        # self.avoidance_vector = self.avoidance_vector_buffer[-1]

        velocity = self.default_velocity
        middle_position = math.degrees(
            self.robot_controller.middle_motor_data["position"][-1]
        )
        omega = middle_position * handle_gain - self.avoidance_vector
        # sd.play(self.exchange_tone) #音
        self.robot_controller.move(velocity, omega)

    def finish(self):
        self.control_timer.cancel()
        while not self.control_timer.finished:
            time.sleep(0.1)
        self.control_timer = None
        self.lidar.finish()
        self.arduino_controller.finish()
        self.robot_controller.release()


class RepeatTimer(threading.Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)
    robot = ControlCenter()

    input()
    del robot
