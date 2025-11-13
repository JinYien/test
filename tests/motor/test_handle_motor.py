from time import sleep

from pykeigan import usbcontroller

from mobile_collab_robot.misc_funcs import get_port_from_hwid, list_ports

list_ports()

MIDDLE_PORT = get_port_from_hwid("DM00KV8D")  # ★
print(MIDDLE_PORT)

dev = usbcontroller.USBController(MIDDLE_PORT, baud=1000000, reconnect=False)
dev.enable_action()
dev.set_speed(10)
dev.run_forward()
sleep(2)
dev.disable_action()
