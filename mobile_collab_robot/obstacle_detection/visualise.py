import numpy as np
import pyqtgraph as pg
from PySide6 import QtWidgets


class LidarVisualise(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.main_layout = QtWidgets.QVBoxLayout(self)

        self.plot_widget = pg.PlotWidget(background="white")
        self.main_layout.addWidget(self.plot_widget)

        self.plot_widget.setAspectLocked(True)
        # self.plot_widget.hideAxis("left")
        # self.plot_widget.hideAxis("bottom")

        # Add polar grid lines
        self.plot_widget.addLine(x=0, pen=pg.mkPen(color=(255, 255, 255, 100), width=1))
        self.plot_widget.addLine(y=0, pen=pg.mkPen(color=(255, 255, 255, 100), width=1))
        # self.plot_widget.getViewBox().invertY(True)
        # self.plot_widget.getViewBox().invertX(True)

        for r in range(0, 6, 1):
            circle = QtWidgets.QGraphicsEllipseItem(-r, -r, r * 2, r * 2)
            circle.setPen(pg.mkPen(color=(255, 255, 255, 100), width=5))
            self.plot_widget.addItem(circle)

        self.detection_region = self.plot_widget.plot(
            fillLevel=0, fillBrush=pg.mkBrush(color=(44, 160, 44, 100))
        )

        self.min_width_lines = [
            self.plot_widget.addLine(pen=pg.mkPen(color=(31, 119, 180, 200), width=1))
            for _ in range(2)
        ]

        self.edge_line = self.plot_widget.plot(
            pen=pg.mkPen(color=(0, 0, 0, 200), width=2)
        )
        self.segment_line = self.plot_widget.plot(
            pen=pg.mkPen(None),
            symbolPen=pg.mkPen(color=(214, 39, 40, 200)),
            symbolBrush=pg.mkBrush(None),
            symbol="o",
            symbolSize=5,
        )

        self.pathway_lines = [
            self.plot_widget.plot(
                pen=pg.mkPen(color=(148, 103, 189, 150), width=5),
            )
            for _ in range(10)
        ]
        self.direction_line = self.plot_widget.plot(
            pen=pg.mkPen(color=(148, 103, 189, 150), width=5)
        )
        self.arrow = pg.ArrowItem(headWidth=0)
        self.plot_widget.addItem(self.arrow)

    def set_scan(self, angles, distances):
        x, y = self.polar_to_cartesian(angles, distances)
        self.edge_line.setData(x, y)

    def set_scan_xy(self, x, y):
        self.edge_line.setData(x, y)

    def set_detection_region(self, x_min, x_max, y_min, y_max):
        self.detection_region.setData(
            [x_min, x_min, x_max, x_max],
            [y_min, y_max, y_max, y_min],
        )

    def set_min_width_line(self, x_min, x_max):
        for l in self.min_width_lines:
            l.setAngle(90)
        self.min_width_lines[0].setValue(x_min)
        self.min_width_lines[1].setValue(x_max)

    @staticmethod
    def polar_to_cartesian(angles, distances):
        theta = np.radians(angles) + np.pi / 2  # +90 degrees to make forward up
        radius = np.array(distances)
        x = -radius * np.cos(theta)
        y = radius * np.sin(theta)
        return x, y

    def set_segments(self, segments):
        angles = np.concat([s[0] for s in segments])
        distances = np.concat([s[1] for s in segments])
        x, y = self.polar_to_cartesian(angles, distances)
        self.segment_line.setData(x, y)

    def set_segments_xy(self, segments_xy):
        x = np.concat([s[0] for s in segments_xy])
        y = np.concat([s[1] for s in segments_xy])
        self.segment_line.setData(x, y)

    def set_pathways(self, pathways):
        for i, line in enumerate(self.pathway_lines):
            if i < len(pathways):
                pathway = pathways[i]
                x, y = self.polar_to_cartesian(pathway[0], pathway[1])
                line = self.pathway_lines[i]
                line.setData(x, y)
            else:
                line.setData()

    def set_pathways_xy(self, pathways):
        for i, line in enumerate(self.pathway_lines):
            if i < len(pathways):
                pathway = pathways[i]
                line = self.pathway_lines[i]
                line.setData(np.array(pathway))
            else:
                line.setData()

    def set_arrow(self, pos, vector, **kwargs):
        length = np.abs(vector)
        if vector > 0:
            angle = 180
        else:
            angle = 0
        self.arrow.setPos(*pos)
        self.arrow.setStyle(angle=angle, tailLen=length, headWidth=10)


if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    widget = LidarVisualise()
    widget.set_scan(np.arange(-120, 120, 10), np.arange(120, 120 * 3, 10))
    widget.show()

    app.exec()
