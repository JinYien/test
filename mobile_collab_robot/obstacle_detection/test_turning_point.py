import logging

import numpy as np
from PySide6 import QtCore, QtWidgets

from mobile_collab_robot.misc_funcs import get_port_from_hwid
from mobile_collab_robot.misc_funcs import setup_logger
from mobile_collab_robot.obstacle_detection.lidar import Lidar, convert_raw_data
from mobile_collab_robot.obstacle_detection.turning_point import TurningPoint
from mobile_collab_robot.obstacle_detection.visualise import LidarVisualise

logger = setup_logger(__name__, logging_level=logging.INFO)


class TestTurningPoint(QtWidgets.QMainWindow):

    def __init__(self, hwid):
        super().__init__()

        self.setWindowTitle("Test Turning Point")

        port = get_port_from_hwid(hwid)
        self.lidar = Lidar(port)

        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update)
        self.update_timer.start(30)
        self.index = 0

        self.visualiser = LidarVisualise(parent=self)
        self.setCentralWidget(self.visualiser)

        self.detection = TurningPoint(
            angles=None,
            distances=None,
        )

    def update(self):
        angles, distances, timestamp = self.lidar.get_scan()  # ★★★★
        angles = np.array(angles)  # ★★★★
        distances = np.array(distances)  # ★★★★

        angles, distances = convert_raw_data(angles, distances, max_distance=6)  # ★★★★
        self.detection.update_data(
            angles,
            distances,
        )
        result = self.detection.run()
        if result:
            logger.info("Turning point detected!")

        self.visualiser.set_scan(angles, distances)
        if result:
            self.visualiser.set_arrow((0, 0), 90, headWidth=50)
        else:
            self.visualiser.set_arrow((0, 0), 0, headWidth=0)

        self.visualiser.show()

    def closeEvent(self, event):
        del self.lidar
        event.accept()


def main():
    app = QtWidgets.QApplication([])

    widget = TestTurningPoint(hwid="VID:PID=15D1:0000")
    widget.show()

    app.exec()


if __name__ == "__main__":
    main()
