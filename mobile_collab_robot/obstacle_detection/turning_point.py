import logging

import numpy as np

from mobile_collab_robot.misc_funcs import setup_logger

logger = setup_logger(__name__, logging_level=logging.INFO)


class TurningPoint:
    def __init__(
        self,
        angles,
        distances,
        min_distance_to_wall=1.5,
        min_distance_to_opening=2.0,
        max_angle=100,
        min_angle=70,
        wall_width=0.45,#0.4
        border=90,
    ):
        self.angles = angles
        self.distances = distances
        self.sampling_angle = None
        self.min_distance_to_wall = min_distance_to_wall
        self.min_distance_to_opening = min_distance_to_opening
        self.max_angle = max_angle
        self.min_angle = min_angle
        self.wall_width = wall_width
        self.border = border

    def update_data(self, angles, distances):
        self.angles = angles
        self.distances = distances
        self.sampling_angle = np.mean(np.diff(self.angles))

    def run(self):
        (r_front_distances, r_rear_distances, r_rear_angles), (
            l_front_distances,
            l_rear_distances,
            l_rear_angles,
        ) = self.get_side_data()
        right_turning = self.judge_corner(
            r_front_distances, r_rear_distances, r_rear_angles
        )
        left_turning = self.judge_corner(
            l_front_distances, l_rear_distances, l_rear_angles
        )
        if left_turning or right_turning:
            if left_turning:
                print("LLLLLLLLLLLLLLLLL")
            else:
                print("RRRRRRRRRRRRRRRRR")
            return True
        else:
            return False

    def get_side_data(self):
        right_mask_angles_range = (self.angles >= self.min_angle) & (
            self.angles <= self.max_angle
        )
        right_mask_border = self.angles <= self.border
        r_front_distances = self.distances[right_mask_angles_range & right_mask_border]
        r_rear_distances = self.distances[right_mask_angles_range & ~right_mask_border]
        r_rear_angles = self.angles[right_mask_angles_range & ~right_mask_border]

        left_mask_angles_range = (self.angles <= -self.min_angle) & (
            self.angles >= -self.max_angle
        )
        left_mask_border = self.angles >= -self.border
        l_front_distances = self.distances[left_mask_angles_range & left_mask_border]
        l_rear_distances = self.distances[left_mask_angles_range & ~left_mask_border]
        l_rear_angles = self.angles[left_mask_angles_range & ~left_mask_border]
        return (
            (r_front_distances, r_rear_distances, r_rear_angles),
            (l_front_distances, l_rear_distances, l_rear_angles),
        )

    def judge_corner(self, front_distances, rear_distances, rear_angles):
        front_bool = front_distances > self.min_distance_to_opening
        front_perc = front_bool.sum() / front_bool.size

        if front_perc < 0.80:
            print("front_distances")
            return False


        rear_bool = rear_distances < self.min_distance_to_wall
        rear_perc = rear_bool.sum() / rear_bool.size
        if rear_perc < 0.75:
            print("rear_distances")
            return False
        
        x, y = self.polar_to_cartesian(rear_angles, rear_distances)
        largest_distance = x.max() - x.min()

        if largest_distance > self.wall_width:
            print("largest_distance")
            return False

        return True

        # front_cnt = 0
        # rear_cnt = 0
        # front_sum = 0
        # rear_sum = 0

        # for i in range(len(front_distances)):
        #     front_sum += front_distances[i]

        #     if front_distances[i] >= self.min_distance_to_opening:
        #         front_cnt += 1
        #         # front_sum += front_distances[i]

        # for j in range(len(rear_distances)):
        #     rear_sum += rear_distances[j]
        #     if (
        #         rear_distances[j] <= self.min_distance_to_wall
        #         and rear_distances[j] >= 0.15
        #     ):
        #         rear_cnt += 1
        #         # rear_sum += rear_distances[j]

        # diff_ave = front_sum / len(front_distances) - rear_sum / len(rear_distances)
        # diff = front_sum - rear_sum
        # # print("diff", diff, diff_ave)

        # logger.debug(
        #     "rear",
        #     rear_cnt,
        #     "front",
        #     front_cnt,
        #     diff_ave,
        #     rear_sum,
        #     front_sum,
        #     rear_sum / len(rear_distances),
        #     front_sum / len(front_distances),
        #     diff,
        # )
        # # print("r",rear_cnt, "f", front_cnt)

        # # if (
        # #     front_cnt >= (len(front_distances) - 5)
        # #     and rear_cnt >= (len(rear_distances) - 5)
        # # ) or (rear_cnt > 5 and diff_ave >= 1 and diff_ave < 1.5):
        # # if (
        # #     front_cnt >= (len(front_distances) )
        # #     and rear_cnt >= (len(rear_distances))
        # # ) or (
        # #     rear_cnt > 10 and diff >= 30 and diff < 80
        # # ) or (
        # #     rear_cnt >= (len(rear_distances)) and diff_ave >= 0.2 and diff_ave <= 0.4
        # # ):  # >=20
        # if (
        #     front_cnt >= (len(front_distances) - 5)
        #     and rear_cnt >= (len(rear_distances))
        #     and diff > 60
        # ) or (rear_cnt >= (len(rear_distances)) and diff_ave >= 0.2 and diff_ave <= 1):
        #     return True
        # else:
        #     return False

    @staticmethod
    def polar_to_cartesian(angles, distances):
        theta = np.radians(angles) + np.pi / 2  # +90 degrees to make forward up
        radius = np.array(distances)
        x = -radius * np.cos(theta)
        y = radius * np.sin(theta)
        return x, y


if __name__ == "__main__":
    scenario = TurningPoint()
