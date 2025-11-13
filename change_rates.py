import time

from pykeigan import usbcontroller

from mobile_collab_robot.misc_funcs import get_port_from_hwid
from mobile_collab_robot.misc_funcs import list_ports


def change_rates(port, baud=115200):
    motor = usbcontroller.USBController(port, baud=baud, debug_mode=False)
    motor.reset_all_registers()
    motor.set_interface(motor.interface_type["USB"] + motor.interface_type["I2C"])
    motor.set_baud_rate(5)
    motor.set_motor_measurement_interval(2)
    motor.save_all_registers()
    print("changed rate...")

    time.sleep(5)
    print("rebooting...")
    motor.reboot()


if __name__ == "__main__":
    MIDDLE_PORT = get_port_from_hwid("DM00KVVM")
    print(MIDDLE_PORT)

    change_rates("COM9")
