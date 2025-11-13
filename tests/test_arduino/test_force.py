from time import sleep

import serial

from mobile_collab_robot.misc_funcs import get_port_from_hwid

ARDUINO_PORT = get_port_from_hwid("8503631343035130C0B1")  #

ser = serial.Serial(ARDUINO_PORT, 115200)
sleep(1)
while True:
    force_xyz = ser.readline()
    try:
        fx, fy, fz = [float(x) for x in force_xyz.decode().strip().split(",")]
        print(fx, fy, fz)
        sleep(0.1)

    except:
        sleep(0.1)
        continue
