from PySide6 import QtWidgets

from mobile_collab_robot.obstacle_detection import (
    LidarVisualise,
    TurningPoint,
    Scenario,
)

scenario = Scenario()
scenario.make_wall((-1.2, 0), (-1.2, -5))

detection = TurningPoint(
    scenario.angles,
    scenario.distances,
)

r_front_distances, r_rear_distances, l_front_distances, l_rear_distances = (
    detection.get_side_data()
)

# check right side
right_turning = detection.judge_corner(r_front_distances, r_rear_distances)

# check left side
left_turning = detection.judge_corner(l_front_distances, l_rear_distances)

print(f"right_turning: {right_turning}, left_turning: {left_turning}")


app = QtWidgets.QApplication([])

visualiser = LidarVisualise()
visualiser.set_scan(scenario.angles, scenario.distances)
visualiser.show()

app.exec()
