import socket
import struct
import time

# from mobile_robot import MobileRobot
from mobile_collab_robot.main2 import ControlCenter

import asyncio

HOST = "192.168.137.106"
PORT = 65444

stop_flag = False


async def client_connected_cb(reader, writer):
    global stop_flag
    data_struct = struct.Struct(">" + "f" * 15 + "c")
    robot = ControlCenter()
    prev_time = time.time()

    while stop_flag is not True:
        if time.time() - prev_time < 0.03:
            time.sleep(0.001)
            continue
        else:
            prev_time = time.time()

        data = [
            robot.robot_controller.robot_data["time"][-1],
            robot.robot_controller.robot_data["state"][-1][0],  # global_x
            robot.robot_controller.robot_data["state"][-1][1],  # global_y
            robot.robot_controller.robot_data["state"][-1][2],  # global_thet
            robot.robot_controller.robot_data["force"][-1],
            robot.robot_controller.robot_data["moment"][-1],
            robot.robot_controller.robot_data["velocity"][-1],
            robot.robot_controller.robot_data["angular_speed"][-1],
            # float(robot.arduino_controller.data["signature"][-1]),
            robot.robot_controller.middle_motor_data["position"][-1],
            robot.robot_controller.middle_motor_data["torque"][-1],
            robot.robot_controller.command_phi_left_dot,
            robot.robot_controller.command_phi_right_dot,
            robot.robot_controller.left_measurements["velocity"],
            robot.robot_controller.right_measurements["velocity"],
            robot.arduino_controller.force_data["Fz"][-1],
            # robot.arduino_controller.data["distance_from_center"][-1],
            # robot.arduino_controller.data["distance_to_object"][-1],
            # LiDARも追加したい
            # robot.judge_pathway.use_angles,
            # robot.judge_pathway.use_distances,
            b"\n",
        ]
        data_packed = data_struct.pack(*data)
        writer.write(data_packed)


async def main():
    server = await asyncio.start_server(client_connected_cb, HOST, PORT)
    print(f"serving on {HOST}, {PORT}")

    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())
    input()

    stop_flag = True
