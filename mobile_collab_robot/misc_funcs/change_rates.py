import argparse
import time

from pykeigan import usbcontroller


def change_rates():
    parser = argparse.ArgumentParser(
        description="Set Keigan motor sampling rate to 100 Hz and baudrate to 1M."
    )
    parser.add_argument("port", type=str, help="port of Keigan motor")
    parser.add_argument("--baud", type=int, default=115200, help="current baudrate")

    args = parser.parse_args()

    motor = usbcontroller.USBController(args.port, baud=args.baud, debug_mode=False)
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
    change_rates()
