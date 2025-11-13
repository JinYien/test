import logging

import numpy as np
from PySide6 import QtCore, QtWidgets

from mobile_collab_robot.misc_funcs import get_port_from_hwid
from mobile_collab_robot.misc_funcs import setup_logger
from mobile_collab_robot.obstacle_detection.lidar import Lidar, convert_raw_data
from mobile_collab_robot.obstacle_detection.obstacle_avoidance import ObstacleAvoidance
from mobile_collab_robot.obstacle_detection.visualise import LidarVisualise

logger = setup_logger(__name__, logging_level=logging.INFO)


class TestAvoidance(QtWidgets.QMainWindow):

    def __init__(self, hwid):
        super().__init__()

        self.setWindowTitle("Test Avoidance")

        port = get_port_from_hwid(hwid)
        self.lidar = Lidar(port)

        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update)
        self.update_timer.start(30)
        self.index = 0

        self.visualiser = LidarVisualise(parent=self)
        self.setCentralWidget(self.visualiser)

        self.detection = ObstacleAvoidance(
            angles=None,
            distances=None,
            barrier_distance=6,
            max_range=3,
            min_range=0,
        )

    def update(self):
        angles, distances, timestamp = self.lidar.get_scan()  # ★★★★

        angles, distances = convert_raw_data(angles, distances, max_distance=6)  # ★★★★
        region = self.detection.get_detection_region()
        x_min, x_max = self.detection.get_min_width()

        self.detection.update_data(
            angles,
            distances,
        )
        result = self.detection.get_optimal_pathway()
        if result != "No pathways":
            pos, direction = result
            logger.info(f"direction: {direction}, pos: {pos}")

        self.visualiser.set_scan(angles, distances)
        self.visualiser.set_detection_region(*region)
        self.visualiser.set_min_width_line(x_min, x_max)
        if self.detection.segments_xy_in_region is not None:
            self.visualiser.set_segments_xy(self.detection.segments_xy_in_region)
        if self.detection.pathways is not None:
            self.visualiser.set_pathways_xy(self.detection.pathways)
        if result != "No pathways":
            self.visualiser.set_arrow(pos, direction, headWidth=10)
        else:
            self.visualiser.set_arrow((0, 0), 0, headWidth=0)
        self.visualiser.show()

    def closeEvent(self, event):
        del self.lidar
        event.accept()


def main():
    app = QtWidgets.QApplication([])

    widget = TestAvoidance(hwid="VID:PID=15D1:0000")
    widget.show()

    app.exec()


if __name__ == "__main__":
    main()
