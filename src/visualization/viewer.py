import pyvista as pv
import numpy as np
from src.core.model import Model
from src.core.results.solution_state import SolutionState

class ModelViewer:
    def __init__(self, model:Model):
        self.model = model
        self.plotter = pv.Plotter()

        nodes = self.model.node
        elements = self.model.element

        self.node_ids = list(nodes.keys())
        self.node_index = {nid: i for i, nid in enumerate(self.node_ids)}

        points = np.array([
            [nodes[node_id].x, nodes[node_id].y, nodes[node_id].z]
            for node_id in self.node_ids
        ])

        lines = []
        for element in elements.values():
            i = self.node_index[element.i.id]
            j = self.node_index[element.j.id]
            lines.append([2, i, j])
        lines = np.array(lines, dtype=int)

        self.point_mesh = pv.PolyData(points)
        self.line_mesh = pv.PolyData(points, lines)

    def add_to_plotter(self):
        self.plotter.add_mesh(
            self.point_mesh,
            color="red",
            point_size=15,
            name="nodes"
        )
        self.plotter.add_mesh(
            self.line_mesh,
            color="black",
            line_width=3,
            style="wireframe",
            name="elements"
        )

    def _show_grid_with_min_bounds(self, scale=0.5):
        # mesh bounds
        xmin, xmax, ymin, ymax, zmin, zmax = self.point_mesh.bounds
        dx = xmax - xmin
        dy = ymax - ymin
        dz = zmax - zmin
        max_dim = max(dx, dy, dz, 1.0)

        # compute minimum thickness
        min_size = scale * max_dim
        xmin -= min_size/2
        xmax += min_size/2
        ymin -= min_size/2
        ymax += min_size/2
        zmin -= min_size/2
        zmax += min_size/2

        self.plotter.show_bounds(
            bounds=(xmin, xmax, ymin, ymax, zmin, zmax),
            grid='back',
            location='outer',
            all_edges=True,
        )
        self.plotter.show_grid()
     
    def show(self):
        self.add_to_plotter()
        self.plotter.add_axes()
        self._show_grid_with_min_bounds(scale=0.5)
        self.plotter.camera_position = [
            (1, 1, 1),   # auto-scaled after reset
            (0, 0, 0),
            (0, 1, 0),   # Y up
        ]
        self.plotter.reset_camera()
        self.plotter.show()

class SolutionStateViewer():
    def __init__(self, state:SolutionState):
        self.state = state
        self.model = state.model
        pass

    def view_model(self):
        # display model
            # self.point_mesh = pv.PolyData(points)
            # self.line_mesh = pv.PolyData(points, lines)
            # releases
            # restraints
            # point loads
            # element loads
        pass

    def view_solution_state(self):
        # display solution state
            # deformed model
                # displaced nodes
                # deformed elements
            # reactions
        pass

    pass
