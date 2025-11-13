import numpy as np


class Scenario:
    def __init__(self) -> None:
        self.angles = np.linspace(-120, 120, 600)
        self.distances = np.ones(self.angles.shape) * 6

    def make_fixed_distance_obstacle(self, obstacle_angle, distance):
        start, end = np.where(
            (self.angles > obstacle_angle[0]) & (self.angles <= obstacle_angle[-1])
        )[0][[0, -1]]
        self.distances[start:end] = distance

    def make_wall(self, point1, point2):
        angle1 = np.degrees(np.arctan2(point1[0], point1[1]))
        angle2 = np.degrees(np.arctan2(point2[0], point2[1]))
        if angle1 > angle2:
            angle1, angle2 = angle2, angle1
        indices = np.where((self.angles > angle1) & (self.angles <= angle2))[0]

        wall_start, wall_end = np.array(point1), np.array(point2)
        vec_point1 = np.array([0, 0])
        for i in indices:
            angle = self.angles[i]
            theta = np.radians(angle)
            vec_point2 = np.array([np.sin(theta), np.cos(theta)])
            x, y = self.get_intersect(wall_start, wall_end, vec_point1, vec_point2)
            self.distances[i] = np.sqrt(x**2 + y**2)

    @staticmethod
    def get_intersect(a1, a2, b1, b2):
        """
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        from https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        """
        s = np.vstack([a1, a2, b1, b2])  # s for stacked
        h = np.hstack((s, np.ones((4, 1))))  # h for homogeneous
        l1 = np.cross(h[0], h[1])  # get first line
        l2 = np.cross(h[2], h[3])  # get second line
        x, y, z = np.cross(l1, l2)  # point of intersection
        if z == 0:  # lines are parallel
            return (float("inf"), float("inf"))
        return (x / z, y / z)


if __name__ == "__main__":
    scenario = Scenario()
    scenario.make_fixed_distance_obstacle((-10, 20), 1.5)
