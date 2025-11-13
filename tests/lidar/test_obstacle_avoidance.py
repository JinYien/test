from PySide6 import QtWidgets

from mobile_collab_robot.obstacle_detection import TestAvoidance

app = QtWidgets.QApplication([])

widget = TestAvoidance(hwid="VID:PID=15D1:0000")
widget.show()

app.exec()
