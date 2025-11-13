import logging

import numpy as np

from mobile_collab_robot.misc_funcs import setup_logger

logger = setup_logger(__name__, logging_level=logging.INFO)


def forward_kinematic_target(forward_vel, omega, a=5, d=20):
    """
    Forward kinematics of a differential drive mobile robot.
    Calculates the wheel speeds from forward and angular velocities.

    :param forward_vel: desired forward velocity in meters per second
    :param omega: desired rotational speed in radians per second
    :param a: wheel radius in m
    :param d: half of distance between the wheels in m
    :return: phi_left_dot, phi_right_dot (* all in radians)
    """
    # phi_left_dot_target = -((d * omega + forward_vel) / a)
    # phi_right_dot_target = (-d * omega + forward_vel) / a
    phi_right_dot_target = -((d * omega + forward_vel) / a)
    phi_left_dot_target = (-d * omega + forward_vel) / a
    return phi_left_dot_target, phi_right_dot_target


def avoid_kinematic_target(forward_vel, omega, a=5, d=20):
    """
    Forward kinematics of a differential drive mobile robot.
    Calculates the wheel speeds from forward and angular velocities.

    :param forward_vel: desired forward velocity in meters per second
    :param omega: desired rotational speed in radians per second
    :param a: wheel radius in m
    :param d: half of distance between the wheels in m
    :return: phi_left_dot, phi_right_dot (* all in radians)
    """
    phi_left_dot_target = -(((d * (omega * 0.6)) * 1.2 + (forward_vel) * 1.2) / a)
    phi_right_dot_target = ((-d * (omega * 0.6)) * 1.2 + (forward_vel) * 1.2) / a
    logger.debug(
        f"{(omega, d * omega, forward_vel, phi_left_dot_target, phi_right_dot_target)}"
    )
    return phi_left_dot_target, phi_right_dot_target


def lidar_kinematic(direction, base_vel=5):
    """
    Forward kinematics of a differential drive mobile robot.
    Calculates the wheel speeds from forward and angular velocities.

    :param forward_vel: desired forward velocity in meters per second
    :param omega: desired rotational speed in radians per second
    :param a: wheel radius in m
    :param d: half of distance between the wheels in m
    :return: phi_left_dot, phi_right_dot (* all in radians)
    """
    # phi_left_dot_target= -(((-X*Gx))+ (2-Y)*Gy)-base_vel
    # phi_right_dot_target = base_vel+((-(-X*Gx))+ (2-Y)*Gy)
    # if X > 0:
    #     phi_left_dot_target= -(((-X*Gx)))-base_vel
    #     phi_right_dot_target = base_vel+((-(-X*Gx))+ (2-Y)*Gy)
    # else:
    #     phi_left_dot_target= -(((-X*Gx))- (2-Y)*Gy)-base_vel
    #     phi_right_dot_target = base_vel+((-(-X*Gx)))
    # if check_lr:
    #     print("=================")
    #     if X >= bace_line:
    #         phi_left_dot_target= (((-(X-bace_line)*Gx)))-base_vel- (2-Y)*Gy
    #         phi_right_dot_target = base_vel+(((-(X-bace_line)*Gx))+ (2-Y)*Gy)
    #     else:
    #         phi_left_dot_target= (((-X*Gx)))-base_vel
    #         phi_right_dot_target = base_vel+((-(-X*Gx))+ (2-Y)*Gy)
    # else:
    #     print("~~~~~~~~~~~~~~~~")
    #     phi_left_dot_target= -(((-X*Gx))- (2-Y)*Gy)-base_vel
    #     phi_right_dot_target = base_vel+((-(-X*Gx)))
    # todo
    if direction > 0:
        phi_left_dot_target = -base_vel - (direction * 10 - 1)
        phi_right_dot_target = base_vel - direction * 1.5
    else:
        phi_left_dot_target = -base_vel + direction * 1.5
        phi_right_dot_target = base_vel - (direction * 10 + 1)

    # phi_left_dot_target = -base_vel - direction
    # phi_right_dot_target = base_vel - direction

    print("▲", phi_left_dot_target, phi_right_dot_target)

    return phi_left_dot_target, phi_right_dot_target


def forward_kinematic(forward_vel, omega, a=5, d=20, saturate=13, lf_correction=0):
    """
    Forward kinematics of a differential drive mobile robot.
    Calculates the wheel speeds from forward and angular velocities.

    :param forward_vel: desired forward velocity in meters per second
    :param omega: desired rotational speed in radians per second
    :param a: wheel radius in m
    :param d: half of distance between the wheels in m
    :return: phi_left_dot, phi_right_dot (* all in radians)
    """
    phi_left_dot = ((-d * omega + forward_vel) / a) - lf_correction
    phi_right_dot = lf_correction - (d * omega + forward_vel) / a
    # phi_left_dot = -((-d * omega + forward_vel) / a)
    # phi_right_dot = (d * omega + forward_vel) / a
    # print("l_v",phi_left_dot, "r_v",phi_right_dot)
    # print("flv:",phi_left_dot,"frv:",phi_right_dot) #速度表示

    # 付け加えてみた（240329）スピード暴走おさえたい
    if phi_left_dot < -saturate:
        phi_left_dot = -saturate
    if phi_right_dot > saturate:
        phi_right_dot = saturate

    return phi_left_dot, phi_right_dot


def inverse_kinematic(phi_left_dot, phi_right_dot, a=5, d=20):
    """
    Forward kinematics of a differential drive mobile robot.
    Calculates the wheel speeds from forward and angular velocities.
    """
    forward_vel = a / 2 * (phi_left_dot + phi_right_dot)
    omega = a * (-phi_left_dot + phi_right_dot) / (2 * d)
    return forward_vel, omega


def force_model(torque_left, torque_right, a=5, d=20):
    force = torque_left / a + torque_right / a
    moment = (torque_left / a - torque_right / a) * d
    return force, moment


class InverseKinematicGlobal:
    """
    Inverse kinematics of a differential drive mobile robot.
    Track position based on wheel velocities
    """

    def __init__(self, a=5, d=20, init_state=None, init_wheels=None):
        """
        :param a: wheel radius in m
        :param d: half of distance between the wheels in m
        :param init_state: starting position and orientation
        :param init_wheels: starting velocity of the wheels in radians per second
        """

        if init_wheels is None:
            init_wheels = [0, 0]  # phi_left_dot, phi_right_dot
        if init_state is None:
            init_state = [0, 0, 0]  # x, y, theta

        self.a = a
        self.d = d
        self.prev_t = 0

        self.state = np.array(init_state)  # x, y, theta
        self.wheel_vel = np.array([init_wheels])  # phi_left_dot, phi_right_dot

        self.time = []
        self.pos = []

        self.record = True

    def update(self, current_t, phi_left_dot, phi_right_dot):
        dt = current_t - self.prev_t
        self.state = self.integrate(self.state, (phi_left_dot, phi_right_dot), dt)

        if self.record:
            self.time.append(current_t)
            self.pos.append(self.state)
        self.prev_t = current_t

    def integrate(self, y, v, dt):
        """RK4 Integrator"""

        theta = y[2]
        k1 = self.RHS(v, theta)
        k2 = self.RHS(v, theta + k1[2] * dt / 2)
        k3 = self.RHS(v, theta + k2[2] * dt / 2)
        k4 = self.RHS(v, theta + k3[2] * dt)
        return y + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    def RHS(self, v, theta):
        phi_left_dot, phi_right_dot = v
        theta_dot = self.a * (-phi_left_dot + phi_right_dot) / (2 * self.d)
        x_dot = self.a * (phi_left_dot + phi_right_dot) * np.cos(theta) / 2
        y_dot = self.a * (phi_left_dot + phi_right_dot) * np.sin(theta) / 2
        return np.array([x_dot, y_dot, theta_dot])


if __name__ == "__main__":
    # simulate
    step = 0.01  # seconds
    conditions = [
        {"end_t": 2, "forward_vel": 0, "omega": np.pi / 4},
        {"end_t": 4, "forward_vel": 5, "omega": 0},
        {"end_t": 6, "forward_vel": 0, "omega": np.pi / 4},
        {"end_t": 8, "forward_vel": 5, "omega": 0},
        {"end_t": 10, "forward_vel": 0, "omega": np.pi / 4},
        {"end_t": 12, "forward_vel": 5, "omega": 0},
        {"end_t": 14, "forward_vel": 0, "omega": np.pi / 4},
        {"end_t": 16, "forward_vel": 5, "omega": 0},
        {"end_t": 20, "forward_vel": -5, "omega": -np.pi / 4},
    ]

    track_pos = InverseKinematicGlobal()
    t = 0
    for c in conditions:
        while t <= c["end_t"]:
            phi_dot_left, phi_dot_right = forward_kinematic(
                c["forward_vel"], c["omega"]
            )
            track_pos.update(t, phi_dot_left, phi_dot_right)
            t += step
