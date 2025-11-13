"""
Obstacle detection
"""

from PySide6 import QtWidgets

from mobile_collab_robot.obstacle_detection import (
    LidarVisualise,
    ObstacleAvoidance,
    Scenario,
)

scenario = Scenario()
scenario.make_fixed_distance_obstacle((-50, -40), 4.3)
# scenario.make_wall((-0.4, 0), (-0.4, 5))
scenario.make_wall((-2, 0), (-2, 6))
scenario.make_fixed_distance_obstacle((-10, 5), 2.5)
scenario.make_fixed_distance_obstacle((40, 55), 2.3)
# scenario.make_fixed_distance_obstacle((, 20), 6)
# scenario.make_fixed_distance_obstacle((10, 15), 3.5)
# scenario.make_fixed_distance_obstacle((-80, -20), 3.5)

detection = ObstacleAvoidance(
    scenario.angles,
    scenario.distances,
    barrier_distance=6,
    max_range=3,
    min_range=0,
)
region = detection.get_detection_region()
x_min, x_max = detection.get_min_width()
detection.update_data(
    scenario.angles,
    scenario.distances,
)
pos, direction = detection.get_optimal_pathway()
print(direction)

app = QtWidgets.QApplication([])

visualiser = LidarVisualise()
visualiser.set_scan(scenario.angles, scenario.distances)
visualiser.set_detection_region(*region)
visualiser.set_min_width_line(x_min, x_max)
if detection.segments_xy_in_region is not None:
    visualiser.set_segments_xy(detection.segments_xy_in_region)
if detection.pathways is not None:
    visualiser.set_pathways_xy(detection.pathways)
visualiser.set_arrow(pos, direction, headWidth=10)
visualiser.show()

app.exec()
