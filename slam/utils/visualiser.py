import vtk
from octreelib.grid import StaticGrid
from octreelib.octree import Octree, OctreeNode

from typing import List

__all__ = ["Visualiser"]

from vtkmodules.vtkCommonColor import vtkNamedColors


class Visualiser:
    @staticmethod
    def draw(grid: StaticGrid):
        actors = Visualiser.__grid_actors(grid)
        Visualiser.__run_visualization(actors)

    @staticmethod
    def __visualize_octree(octree: Octree):
        actors = Visualiser.__octree_actors(octree)
        Visualiser.__run_visualization(actors)

    @staticmethod
    def __run_visualization(actors: List[vtk.vtkActor]):
        renderer = vtk.vtkRenderer()
        renderer.SetBackground(0.5, 0.6, 0.8)

        for actor in actors:
            renderer.AddActor(actor)

        # Create a render window
        render_window = vtk.vtkRenderWindow()
        render_window.AddRenderer(renderer)
        render_window.SetSize(1600, 1200)

        # Create an interactor for user interaction
        interactor = vtk.vtkRenderWindowInteractor()
        interactor.SetRenderWindow(render_window)

        # Create an interactor style for panning and rotating the view
        style = vtk.vtkInteractorStyleTrackballCamera()
        interactor.SetInteractorStyle(style)

        # Start the interactor
        interactor.Initialize()
        interactor.Start()

    @staticmethod
    def __octree_actors(octree: Octree):
        def octree_node_actors(node: OctreeNode):
            if node.has_children:
                return sum([octree_node_actors(child) for child in node.children], [])
            bounds = [
                node.bounding_box[0][0],
                node.bounding_box[1][0],
                node.bounding_box[0][1],
                node.bounding_box[1][1],
                node.bounding_box[0][2],
                node.bounding_box[1][2],
            ]
            cube_source = vtk.vtkCubeSource()
            cube_source.SetBounds(bounds)

            cube_mapper = vtk.vtkPolyDataMapper()
            cube_mapper.SetInputConnection(cube_source.GetOutputPort())

            cube_actor = vtk.vtkActor()
            cube_actor.SetMapper(cube_mapper)
            cube_actor.GetProperty().SetRepresentationToWireframe()
            cube_actor.GetProperty().SetLineWidth(2.0)

            points = vtk.vtkPoints()
            cells = vtk.vtkCellArray()
            for point in node.points:
                point_id = points.InsertNextPoint(point[:])
                cells.InsertNextCell(1)
                cells.InsertCellPoint(point_id)

                points.Modified()
                cells.Modified()

            input_data = vtk.vtkPolyData()
            input_data.SetPoints(points)
            input_data.SetVerts(cells)

            points_mapper = vtk.vtkPolyDataMapper()
            points_mapper.SetInputData(input_data)

            colors = vtkNamedColors()
            points_actor = vtk.vtkActor()
            points_actor.SetMapper(points_mapper)
            points_actor.GetProperty().SetColor(colors.GetColor3d("Green"))
            points_actor.GetProperty().SetPointSize(3)

            return [cube_actor, points_actor]

        return octree_node_actors(octree.root)

    @staticmethod
    def __grid_actors(grid: StaticGrid):
        return sum(
            [Visualiser.__octree_actors(octree) for octree in grid.octrees.values()], []
        )
