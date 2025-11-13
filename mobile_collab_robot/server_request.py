import socket
import struct
import time

import numpy as np

from mobile_collab_robot.main2 import ControlCenter
from mobile_collab_robot.misc_funcs import pack_frame

HOST = "0.0.0.0"
PORT = 65444
# PORT = 54333

live_data_variables = {
    "time": "robot.robot_controller.robot_data['time'][-1]",
    "global_x": "robot.robot_controller.robot_data['state'][-1][0]",
    "global_y": "robot.robot_controller.robot_data['state'][-1][1]",
    "global_theta": "robot.robot_controller.robot_data['state'][-1][2]",
    "force": "robot.robot_controller.robot_data['force'][-1]",
    "moment": "robot.robot_controller.robot_data['moment'][-1]",
    "velocity": "robot.robot_controller.robot_data['velocity'][-1]",
    "angular_speed": "robot.robot_controller.robot_data['angular_speed'][-1]",
    "position": "robot.robot_controller.middle_motor_data['position'][-1]",
    "torque": "robot.robot_controller.middle_motor_data['torque'][-1]",
    "command_left_wheel": "robot.robot_controller.command_phi_left_dot",
    "command_right_wheel": "robot.robot_controller.command_phi_right_dot",
    "measure_left_wheel": "robot.robot_controller.left_measurements['velocity']",
    "measure_right_wheel": "robot.robot_controller.right_measurements['velocity']",
    "force_sensor": "robot.arduino_controller.force_data['Fz'][-1]",
    "Mx": "robot.arduino_controller.force_data['Mx'][-1]",
    "My": "robot.arduino_controller.force_data['My'][-1]",
    "obstacle_position_x": "robot.obstacle_position[0]",
    "obstacle_position_y": "robot.obstacle_position[1]",
    "avoidance_vector": "robot.avoidance_vector",
}
stop_flag = False

robot = ControlCenter()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    s.bind((HOST, PORT))
    s.listen(1)
    print("listening at", HOST, PORT)
    conn, addr = s.accept()
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
    prev_time = time.time()
    data_struct = struct.Struct(">" + "f" * len(live_data_variables) + "c")

    with conn:
        print(f"Connected by {addr}")
        while True:
            request = conn.recv(1)
            if request == b"\x01":  # requesting live data
                data = [eval(code) for code in live_data_variables.values()]
                data.append(b"\n")
                data_packed = data_struct.pack(*data)
                conn.send(data_packed)

            elif request == b"\x02":  # requesting lidar data
                lidar_data = np.array([robot.angles, robot.distances])
                data_bytes = pack_frame(lidar_data)
                conn.send(data_bytes)
                
            elif request == b"\x03":  # change distance gain
                data = conn.recv(1)
                distance_gain = struct.Struct(">f").unpack(data)
                robot.distance_gain = distance_gain

            elif request == b"\x10":  # finish
                print("finished")
                conn.send(b"\x10")
                robot.finish()
                break

del robot
