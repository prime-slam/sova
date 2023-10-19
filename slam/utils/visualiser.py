import k3d
import numpy as np
from octreelib.grid import StaticGrid
from octreelib.octree import Octree, OctreeNode

import itertools
from typing import List

__all__ = ["Visualiser"]


class Visualiser:
    @staticmethod
    def draw(grid: StaticGrid, filename: str):
        plot = k3d.Plot()

        points = Visualiser.__get_points(grid)
        plot += k3d.points(
            positions=points,
            point_size=0.1,
            shader="3d",
        )

        vertices = Visualiser.__get_voxels_vertices(grid)
        for vertex in vertices:
            plot += k3d.lines(
                vertices=vertex,
                indices=[
                    [0, 2, 2, 6, 6, 4, 4, 0],  # Yeah, that's weird
                    [0, 1, 1, 5, 5, 4, 4, 0],  # but I didn't invent other way
                    [0, 1, 1, 3, 3, 2, 2, 0],  # to trace vertices
                    [1, 3, 3, 7, 7, 5, 5, 1],
                    [2, 3, 3, 7, 7, 6, 6, 2],
                    [4, 5, 5, 7, 7, 6, 6, 4],
                ],
                width=0.01,
                color=0xFF0000,
                indices_type="segment",
            )

        with open(filename, "w") as f:
            f.write(plot.get_snapshot())

    @staticmethod
    def __get_points(grid: StaticGrid) -> np.ndarray:
        points = np.empty((0, 3))

        for ind in range(len(grid.octrees.values())):
            for point in grid.get_points(ind):
                points = np.append(points, [point], axis=0)

        return points

    @staticmethod
    def __get_voxels_vertices(grid: StaticGrid):
        vertices = []
        for octree in grid.octrees.values():
            voxel_vertices = Visualiser.__get_octree_voxels(octree)
            vertices.extend(voxel_vertices)

        return np.asarray(vertices)

    @staticmethod
    def __get_octree_voxels(octree: Octree) -> List:
        def get_children_bounds(node: OctreeNode) -> List:
            corners = []
            if node.has_children:
                for child in node.children:
                    corners.extend(get_children_bounds(child))

                return corners

            return [
                [
                    node.corner + offset
                    for offset in itertools.product([0, node.edge_length], repeat=3)
                ]
            ]

        return get_children_bounds(octree.root)
