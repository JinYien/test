import logging
import threading
import time
from collections import deque
from math import radians

from pykeigan import usbcontroller

from mobile_collab_robot.misc_funcs import setup_logger
from mobile_collab_robot.robot_control.model import (
    forward_kinematic_target,
    inverse_kinematic,
    force_model,
    InverseKinematicGlobal,
)

logger = setup_logger(__name__, logging_level=logging.INFO)


class RobotController:
    def __init__(
        self,
        left_port="/dev/ttyUSB0",
        right_port="/dev/ttyUSB1",
        middle_port="/dev/ttyUSB2",
        baud=1000000,
    ):
        self.command_phi_left_dot = 0
        self.command_phi_right_dot = 0

        self.wheel_radius = 60e-3
        self.distance_between_wheels = 310e-3
        self.global_model = InverseKinematicGlobal(
            a=self.wheel_radius, d=self.distance_between_wheels / 2
        )
        self.global_model.record = False

        self.left_measurements = None
        self.right_measurements = None
        self.middle_measurements = None  # ★

        self.left_arrived = False
        self.right_arrived = False
        self.middle_arrived = False  # ★

        data_length = 600
        self.left_motor_data = {
            x: deque(maxlen=data_length)
            for x in ["received_unix_time", "position", "velocity", "torque"]
        }
        self.right_motor_data = {
            x: deque(maxlen=data_length)
            for x in ["received_unix_time", "position", "velocity", "torque"]
        }

        self.middle_motor_data = {  # ★
            x: deque(maxlen=data_length)
            for x in ["received_unix_time", "position", "velocity", "torque"]
        }

        self.robot_data = {
            x: deque(maxlen=data_length)
            for x in ["time", "state", "force", "moment", "velocity", "angular_speed"]
        }
        self.lock = threading.Lock()

        self.motor_left = self.connect_motor(left_port, baud)
        self.motor_right = self.connect_motor(right_port, baud)
        self.motor_middle = self.connect_motor(middle_port, baud)
        logger.info(self.motor_left)
        logger.info(self.motor_right)
        logger.info(self.motor_middle)

        self.motor_left.on_motor_measurement_value_cb = self.motor_left_callback
        self.motor_right.on_motor_measurement_value_cb = self.motor_right_callback
        self.motor_middle.on_motor_measurement_value_cb = (
            self.motor_middle_callback
        )  # ★

        self.start_time = time.time()
        self.middle_free()

    @staticmethod
    def connect_motor(port="/dev/ttyUSB0", baud=1000000):
        dev = usbcontroller.USBController(port, baud=baud, reconnect=False)
        dev.enable_action()
        dev.reset_all_pid()
        dev.set_curve_type(0)
        dev.preset_position(0)  # 現在位置の座標を0に設定
        dev.read_serial_polling_time = 0
        return dev
    
    def set_max_torque(self, max_torque):
        self.motor_left.set_max_torque(max_torque)
        self.motor_right.set_max_torque(max_torque)

    def setup_middle_motor(self, gain, angle_threshold_deg=0.01):
        self.motor_middle.set_position_p(gain)
        self.motor_middle.set_position_i(0)
        self.motor_middle.set_position_d(0)
        self.motor_middle.set_pos_control_threshold(radians(angle_threshold_deg))
        self.motor_middle.set_max_torque(0.3)
        self.motor_middle.move_to_pos(0)

    def motor_left_callback(self, measurements):
        self.left_measurements = measurements
        for k, v in self.left_measurements.items():
            self.left_motor_data[k].append(v)
        self.left_arrived = True
        self.update_model()

    def motor_right_callback(self, measurements):
        self.right_measurements = measurements
        for k, v in self.right_measurements.items():
            self.right_motor_data[k].append(v)
        self.right_arrived = True
        self.update_model()

    def motor_middle_callback(self, measurements):
        self.middle_measurements = measurements
        for k, v in self.middle_measurements.items():
            self.middle_motor_data[k].append(v)
        self.middle_arrived = True
        # self.update_model()

    def update_model(self):
        if self.left_arrived and self.right_arrived:
            current_t = time.time() - self.start_time
            self.global_model.update(
                current_t,
                self.left_measurements["velocity"],
                -self.right_measurements["velocity"],
            )
            force, moment = force_model(
                self.left_measurements["torque"],
                -self.right_measurements["torque"],
                a=self.wheel_radius,
                d=self.distance_between_wheels / 2,
            )
            forward_vel, omega = inverse_kinematic(
                self.left_measurements["velocity"],
                -self.right_measurements["velocity"],
                a=self.wheel_radius,
                d=self.distance_between_wheels / 2,
            )
            self.right_arrived = False
            self.left_arrived = False

            # with self.lock:
            self.robot_data["time"].append(current_t)
            self.robot_data["state"].append(self.global_model.state)
            self.robot_data["force"].append(force)
            self.robot_data["moment"].append(moment)
            self.robot_data["velocity"].append(forward_vel)
            self.robot_data["angular_speed"].append(omega)

    def stop(self):
        self.motor_right.stop_motor()
        self.motor_left.stop_motor()
        self.command_phi_left_dot = 0
        self.command_phi_right_dot = 0
        # self.motor_middle.hold_torque(0.0)

    def stop_free(self):
        self.motor_right.hold_torque(0.0)
        self.motor_left.hold_torque(0.0)
        self.command_phi_left_dot = 0
        self.command_phi_right_dot = 0
        # self.motor_middle.hold_torque(0.0)    # ここ

    def middle_free(self):
        self.motor_middle.hold_torque(0.0)

    def move(self, target_velocity, target_omega):
        phi_left_dot, phi_right_dot = forward_kinematic_target(
            target_velocity,
            target_omega,
            a=self.wheel_radius,
            d=self.distance_between_wheels / 2,
        )
        

        # 暴走チェック
        max_phi_dot = 14
        if phi_left_dot > max_phi_dot:
            phi_left_dot = max_phi_dot
        elif phi_left_dot < -max_phi_dot:
            phi_left_dot = -max_phi_dot
        if phi_right_dot > max_phi_dot:
            phi_right_dot = max_phi_dot
        elif phi_right_dot < -max_phi_dot:
            phi_right_dot = -max_phi_dot

        self.command_phi_left_dot = phi_left_dot
        self.command_phi_right_dot = phi_right_dot

        print("l",self.command_phi_left_dot,"r",self.command_phi_right_dot)

        self.motor_left.run_at_velocity(self.command_phi_left_dot)
        self.motor_right.run_at_velocity(self.command_phi_right_dot)

    def release(self):
        self.motor_left.stop_motor()
        self.motor_right.stop_motor()
        self.motor_middle.stop_motor()

        self.motor_left.disable_action()
        self.motor_right.disable_action()
        self.motor_middle.disable_action()

        self.motor_left.disconnect()
        self.motor_right.disconnect()
        self.motor_middle.disconnect()

        logger.info("Finished")


if __name__ == "__main__":
    robot = RobotController(
        left_port="/dev/ttyUSB0",
        right_port="/dev/ttyUSB1",
        middle_port="/dev/ttyUSB2",  # ★
    )
    robot.stop_free()
    input()
