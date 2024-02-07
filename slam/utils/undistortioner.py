import math

import mrob
import numpy as np
import open3d as o3d

from slam.typing import Array4x4

__all__ = ["Undistortioner"]


class Undistortioner:
    def __init__(self, segments_number: int):
        self._segments_number = segments_number

    def __call__(
        self,
        initial_point_cloud: o3d.geometry.PointCloud,
        initial_pose: Array4x4[float],
        target_point_cloud: o3d.geometry.PointCloud,
        target_pose: Array4x4[float],
    ):
        initial_point_cloud = o3d.geometry.PointCloud(initial_point_cloud)
        target_point_cloud = o3d.geometry.PointCloud(target_point_cloud)

        undistorted_point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector())

        self._segments_number += 1
        coefficients = np.linspace(0, 1, self._segments_number)
        initial_angle = np.pi
        sectors = np.linspace(
            initial_angle, initial_angle - 2 * np.pi, self._segments_number
        )

        initial_point_cloud.transform(initial_pose)
        target_point_cloud.transform(target_pose)

        coefficients = np.roll(coefficients, -len(coefficients) // 2)

        for i in range(1, self._segments_number):
            segment = self._cut_segment(initial_point_cloud, sectors[i], initial_angle)

            transform_matrix = self._get_intermediate_transform(
                initial_pose, target_pose, coefficients[i - 1]
            )

            undistorted_point_cloud += segment.transform(transform_matrix)
            initial_angle = sectors[i]

        return undistorted_point_cloud

    def _get_intermediate_transform(
        self,
        initial_pose: Array4x4[float],
        target_pose: Array4x4[float],
        coefficient: float,
    ):
        dxi = mrob.geometry.SE3(target_pose @ np.linalg.inv(initial_pose)).Ln()
        return mrob.geometry.SE3(coefficient * dxi).T() @ initial_pose

    def _cut_segment(
        self, point_cloud: o3d.geometry.PointCloud, theta_min: float, theta_max: float
    ):
        points = np.asarray(point_cloud.points)
        thetas = np.asarray([math.atan2(p[1], p[0]) for p in points])

        mask_roi = (thetas < theta_max) & (thetas >= theta_min)
        trues = mask_roi.sum()

        pcd = o3d.geometry.PointCloud()
        if trues > 0:
            points = points[mask_roi]
            pcd.points = o3d.utility.Vector3dVector(points)

        return pcd
