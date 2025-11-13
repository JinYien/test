import logging

import numpy as np

from mobile_collab_robot.misc_funcs import setup_logger

logger = setup_logger(__name__, logging_level=logging.INFO)


class FrontObstacle:
    def __init__(
        self,
        angles,
        distances,
        position,
        range_angle = 10,
        min_distance_to_obstacle=1.5,
    ):
        self.angles = angles
        self.distances = distances
        self.position = position
        self.range_angle = range_angle 
        self.min_distance_to_obstacle=min_distance_to_obstacle
        self.sampling_angle = None


    def update_data(self, angles, distances, position):
        self.angles = angles
        self.distances = distances
        self.position = position
        self.sampling_angle = np.mean(np.diff(self.angles))
        # print("m_pos",self.position)

    def run(self):
        front_distances = (
            self.get_front_data()
        )
        front_obstacle = self.judge_front_obstacle(front_distances)
        # print("f",front_obstacle)
        return front_obstacle



    def get_front_data(self):
        front_mask_angles_range = (self.angles >= self.position - self.range_angle) & (
            self.angles <= self.position + self.range_angle
        )
   
        front_distances = self.distances[front_mask_angles_range]
        # print(front_distances)
       
        return front_distances

    def judge_front_obstacle(self, front_distances):
        cnt = 0
    
        for i in range(len(front_distances)):

            if front_distances[i] <= self.min_distance_to_obstacle:
                cnt += 1

                # rear_sum += rear_distances[j]


        logger.debug(
            "",
            cnt,
            "front",
            len(front_distances)

        )


        if (
            cnt >= len(front_distances)-10
        ): 
            return True
        else:
            return False

    @staticmethod
    def polar_to_cartesian(angles, distances):
        theta = np.radians(angles) + np.pi / 2  # +90 degrees to make forward up
        radius = np.array(distances)
        x = -radius * np.cos(theta)
        y = radius * np.sin(theta)
        return x, y


if __name__ == "__main__":
    scenario = FrontObstacle()
