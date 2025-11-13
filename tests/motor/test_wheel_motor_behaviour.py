from time import sleep

from pykeigan import usbcontroller

from mobile_collab_robot.misc_funcs import get_port_from_hwid

LEFT_PORT = get_port_from_hwid("DM00KJR5")
RIGHT_PORT = get_port_from_hwid("DM00KWZP")

dev_l = usbcontroller.USBController(LEFT_PORT, baud=1000000, reconnect=False)
dev_l.enable_action()
dev_l.stop_motor()

dev_r = usbcontroller.USBController(RIGHT_PORT, baud=1000000, reconnect=False)
dev_r.enable_action()
dev_r.hold_torque(0.0)

input()
dev_l.disable_action()
dev_r.disable_action()
