import logging

import serial.tools.list_ports


def list_ports():
    for port, desc, hwid in serial.tools.list_ports.comports():
        print(f"port: {port}, desc: {desc}, hwid: {hwid}")


def get_port_from_hwid(hw_ser):
    for port, desc, hwid in serial.tools.list_ports.comports():
        if hw_ser in hwid:
            logging.info(f"Found at {port}")
            return port
    else:
        logging.info("Not port found!")
