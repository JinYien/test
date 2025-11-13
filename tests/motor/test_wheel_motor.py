from time import sleep

from pykeigan import usbcontroller

from mobile_collab_robot.misc_funcs import get_port_from_hwid

LEFT_PORT = get_port_from_hwid("DM00KJR5")
RIGHT_PORT = get_port_from_hwid("DM00KWZP")

dev = usbcontroller.USBController(LEFT_PORT, baud=1000000, reconnect=False)
dev.enable_action()
dev.set_speed(10)
dev.run_forward()
sleep(2)
dev.disable_action()

dev = usbcontroller.USBController(RIGHT_PORT, baud=1000000, reconnect=False)
dev.enable_action()
dev.set_speed(10)
dev.run_forward()
sleep(2)
dev.disable_action()
