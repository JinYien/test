import logging

import numpy as np

from mobile_collab_robot.misc_funcs import setup_logger

logger = setup_logger(__name__, logging_level=logging.INFO)


class ObstacleAvoidance:
    def __init__(
        self,
        barrier_distance=2,  # 3
        angle_range=55,  # 3
        min_width=0.5,  # 0.8
        scan_region=(-0.55, 0.55, 0, 3),#(-0.6, 0.6, 0.1, 3),
    ):
        self.angles = None
        self.distances = None
        self.sampling_angle = None
        self.max_distance = 6
        self.scan_region = scan_region

        self.barrier_distance = barrier_distance
        self.angle_range = angle_range
        self.min_distance_between_segments = 0.3  # degree
        self.min_width = min_width

        self.segments = None
        self.pathways = None

    def update_data(self, angles, distances):
        mask = (angles > -self.angle_range) & (angles < self.angle_range)

        self.angles = angles[mask]
        self.distances = distances[mask]
        self.sampling_angle = np.mean(np.diff(self.angles))

    def get_obstacles_segments(self, barrier_distance=None):
        """
        returns a list of segments
         - each segment contain an angle array and a distance array
        """
        if barrier_distance is not None:
            # print(barrier_distance)
            self.barrier_distance = barrier_distance
        mask = self.distances < self.barrier_distance

        barrier_angles = self.angles[mask]
        barrier_distances = self.distances[mask]
        if barrier_angles.size == 0:
            return "No obstacles"

        (split_at,) = np.where(
            (np.diff(barrier_angles) > self.sampling_angle * 1.5)
            | (np.diff(barrier_distances) > self.min_distance_between_segments)
        )
        if len(split_at) < 1:
            return [[barrier_angles, barrier_distances]]  # return if no gap in between
        split_at += 1

        segments = [(barrier_angles[: split_at[0]], barrier_distances[: split_at[0]])]
        for i in range(len(split_at) - 1):
            segments.append(
                (
                    barrier_angles[split_at[i] : split_at[i + 1]],
                    barrier_distances[split_at[i] : split_at[i + 1]],
                )
            )
        segments.append(
            (barrier_angles[split_at[-1] :], barrier_distances[split_at[-1] :])
        )

        segments_in_region = []
        segments_xy = self.get_obstacles_segments_xy(segments)
        for s_xy, seg in zip(segments_xy, segments):
            x, y = s_xy
            x_min, x_max, y_min, y_max = self.scan_region
            mask = (x > x_min) & (x < x_max) & (y > y_min) & (y < y_max)
            if mask.any():
                segments_in_region.append(seg)

        self.segments = segments_in_region
        if segments_in_region == []:
            segments_in_region = "No obstacles"
        return segments_in_region

    def get_obstacles_segments_xy(self, segments):
        segments_xy = []
        for s in segments:
            angles, distances = s
            x, y = self.polar_to_cartesian(angles, distances)
            segments_xy.append((x, y))
        self.segments_xy = segments_xy
        return segments_xy

    def get_open_pathways(self, segments=None):
        """
        returns a list of open pathways
         - each pathway contains 4 values, the start angle, end angle, and distance at start of pathway and distance at end of pathway
        """

        if segments is None:
            segments = self.get_obstacles_segments()

        if segments == "No obstacles":
            return "No obstacles"

        open_pathways = []

        # if first segment is not from start angle of sensor
        first_segment_angles, first_segment_distances = segments[0]
        if first_segment_angles[0] > self.angles.min():
            distance = first_segment_distances.min()
            open_pathways.append(
                (
                    (
                        self.angles.min(),
                        first_segment_angles[0],
                    ),
                    (
                        self.max_distance,
                        distance,
                    ),
                )
            )

        for i in range(len(segments) - 1):
            segment_angles, segment_distances = segments[i]
            next_segment_angles, next_segment_distances = segments[i + 1]
            open_pathways.append(
                (
                    (
                        segment_angles[-1],
                        next_segment_angles[0],
                    ),
                    (
                        segment_distances.min(),
                        next_segment_distances.min(),
                    ),
                )
            )

        last_segment_angles, last_segment_distances = segments[-1]

        if last_segment_angles[-1] < self.angles.max():
            distance = last_segment_distances.min()
            open_pathways.append(
                (
                    (
                        last_segment_angles[-1],
                        self.angles.max(),
                    ),
                    (
                        distance,
                        self.max_distance,
                    ),
                )
            )

        self.pathways = open_pathways
        # print(open_pathways)
        if len(open_pathways) == 0:
            return "No pathways"
        return open_pathways

    def get_optimal_pathway(self, pathways=None):
        if pathways is None:
            pathways = self.get_open_pathways()

        # 基本的なケース（障害物なし，通り道無し）
        if pathways == "No obstacles":
            return "No obstacles"
        elif pathways == "No pathways":
            return "No pathways"

        # 通り道幅のチェック
        usable_pathways = []
        for pathway in pathways:
            x, y = self.polar_to_cartesian(*pathway)
            point1 = np.array([x[0], y[0]])
            point2 = np.array([x[1], y[1]])
            width = np.linalg.norm(point1 - point2)
            if width > self.min_width:
                usable_pathways.append(pathway)
        if len(usable_pathways) == 0:
            return "No pathways"

        # 通り道が中心線に跨ぐチェック
        for pathway in usable_pathways:
            (start_angle, end_angle), (start_distance, end_distance) = pathway
            if start_angle <= 0 <= end_angle:
                x, y = self.polar_to_cartesian(
                    (start_angle, end_angle), (start_distance, end_distance)
                )
                if x[0] <= -self.min_width / 2 and x[1] >= self.min_width / 2:  # ★★
                    print("!!!!!!!!!!!")
                    return "No obstacles"
                if np.abs(start_angle) > np.abs(end_angle):
                    return end_angle, end_distance, False
                    # returns obstacle position, direction to avoid
                else:
                    return start_angle, start_distance, True

        # 中心線に近い通り道を選ぶ
        angles_from_centre = []
        for pathway in usable_pathways:
            (start_angle, end_angle), (start_distance, end_distance) = pathway
            # 中心線に跨いでいない前提
            if end_angle < 0:
                angles_from_centre.append(np.abs(end_angle))
            else:
                angles_from_centre.append(np.abs(start_angle))
        print(angles_from_centre)
        index = np.argmin(angles_from_centre)
        print(index)
        selected_pathway = pathways[index]

        (start_angle, end_angle), (start_distance, end_distance) = (
            selected_pathway  # pathway
        )
        if end_angle < 0:
            return end_angle, end_distance, False
        else:
            return start_angle, start_distance, True

    @staticmethod
    def polar_to_cartesian(angles, distances):
        theta = np.radians(angles) + np.pi / 2  # +90 degrees to make forward up
        radius = np.array(distances)
        x = -radius * np.cos(theta)
        y = radius * np.sin(theta)
        return x, y


if __name__ == "__main__":
    scenario = ObstacleAvoidance()
