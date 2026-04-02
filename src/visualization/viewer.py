import pyvista as pv
import numpy as np
from src.core.results.solution_state import SolutionState
from src.utils import global_variables as gv

DOF_MAP = {
    gv.UX: ("UX", np.array([1, 0, 0]), '#f96982', "translation"),
    gv.UY: ("UY", np.array([0, 1, 0]), '#96ef6d', "translation"),
    gv.UZ: ("UZ", np.array([0, 0, 1]), '#6ec8fa', "translation"),
    gv.RX: ("RX", np.array([1, 0, 0]), '#f9a9b6', "rotation"),
    gv.RY: ("RY", np.array([0, 1, 0]), '#c3efb0', "rotation"),
    gv.RZ: ("RZ", np.array([0, 0, 1]), '#a8d9f5', "rotation"),
}

class SolutionStateViewer():
    """
    Object used to view models
    """
    def __init__(self, state:SolutionState, deformation_scale:float=500.0):
        self.model = state.model
        self.displacements = state.displacements

        self.plotter = pv.Plotter()
        self.bounds = None
        self.max_dim = None

        self.nodes = self.model.node
        self.elements = self.model.element

        # Visualization
        self.node_ids = list(self.nodes.keys())
        self.node_index = {nid: i for i, nid in enumerate(self.node_ids)}

        # Original Model
        points_list = []
        for node_id in self.node_ids:
            points_list.append([
                self.nodes[node_id].x,
                self.nodes[node_id].y,
                self.nodes[node_id].z
            ])
        self.points = np.array(points_list)

        lines_list = []
        for element in self.elements.values():
            lines_list.append([2, self.node_index[element.i.id], self.node_index[element.j.id]])
        self.lines = np.array(lines_list, dtype=int)

        # Deformed Model
        deformed_points_list = []
        for node_id in self.node_ids:
            node_global_index_dict = self.nodes[node_id].dofs
            x_disp = self.displacements[node_global_index_dict[gv.UX]]
            y_disp = self.displacements[node_global_index_dict[gv.UY]]
            z_disp = self.displacements[node_global_index_dict[gv.UZ]]

            deformed_points_list.append([
                self.nodes[node_id].x + x_disp*deformation_scale,
                self.nodes[node_id].y + y_disp*deformation_scale,
                self.nodes[node_id].z + z_disp*deformation_scale
            ])
        self.deformed_points = np.array(deformed_points_list)

        # use interpolation so elements consider rotation
        deformed_lines_list = []
        for element in self.elements.values():
            deformed_lines_list.append([2, self.node_index[element.i.id], self.node_index[element.j.id]])
        self.deformed_lines = np.array(deformed_lines_list, dtype=int)

    def _add_node_mesh(self):
        point_mesh = pv.PolyData(self.points)
        self.plotter.add_mesh(
            point_mesh,
            color="#606060",
            point_size=18,
            name="nodes",
            lighting=False
        )
        self.bounds = point_mesh.bounds
        xmin, xmax, ymin, ymax, zmin, zmax = self.bounds
        self.max_dim = max(xmax - xmin, ymax - ymin, zmax - zmin, 1.0) 

    def _add_element_mesh(self):
        line_mesh = pv.PolyData(self.points, self.lines)
        self.plotter.add_mesh(
            line_mesh,
            color="#d8cfc6",
            line_width=6,
            style="wireframe",
            name="elements",
            lighting=False
        )

    def _add_deformed_node_mesh(self):
        deformed_point_mesh = pv.PolyData(self.deformed_points)
        self.plotter.add_mesh(
            deformed_point_mesh,
            color = "#808059",
            point_size=18,
            name="deformed_nodes",
            lighting=False
        )

    def _add_deformed_element_mesh(self, deformation_opacity:float=0.7):
        deformed_line_mesh = pv.PolyData(self.deformed_points, self.deformed_lines)
        self.plotter.add_mesh(
            deformed_line_mesh,
            color="#f0ab59",
            line_width=6,
            style="wireframe",
            name="deformed_elements",
            opacity=deformation_opacity,
            lighting=False
        )

    def _add_restraints_mesh(self):
        scale = 1.0
        for node in self.nodes.values():
            if not node.restraints:     
                continue
            position = np.array([node.x, node.y, node.z])

            for dof, restrained in node.restraints.items():
                if not restrained:
                    continue
                
                name, direction, color, dof_type = DOF_MAP[dof]

                if dof_type == "translation":
                    cone_height = 0.05*self.max_dim * scale
                    cone_radius = 0.0125*self.max_dim * scale

                    cone = pv.Cone(
                        direction=-direction,
                        height=cone_height,
                        radius=cone_radius,
                        resolution=6,
                        center=position+direction*cone_height/2
                    )
                    self.plotter.add_mesh(cone, color=color, lighting=False)

                elif dof_type == "rotation":
                    ring_radius_x = 0.03*self.max_dim * scale
                    ring_radius_y = 0.035*self.max_dim * scale
                    ring_radius_z = 0.04*self.max_dim * scale

                    cross_section_radius_x = 0.0025*self.max_dim * scale
                    cross_section_radius_y = 0.0025*self.max_dim * scale
                    cross_section_radius_z = 0.0025*self.max_dim * scale

                    # rotation about x
                    if np.array_equal(direction, np.array([1, 0, 0])) :
                        ring = pv.ParametricTorus(
                            ringradius=ring_radius_x,
                            crosssectionradius=cross_section_radius_x,
                            center = position
                        )
                        ring.rotate_y(angle=90, point=position,inplace=True)

                    # rotation about y
                    elif np.array_equal(direction, np.array([0, 1, 0])) :
                        ring = pv.ParametricTorus(
                            ringradius=ring_radius_y,
                            crosssectionradius=cross_section_radius_y,
                            center = position
                        )
                        ring.rotate_x(angle=90, point=position, inplace=True)

                    # rotation about z
                    else:                        
                        ring = pv.ParametricTorus(
                            ringradius=ring_radius_z,
                            crosssectionradius=cross_section_radius_z,
                            center = position
                        )

                    self.plotter.add_mesh(ring, color=color, lighting=False)
                else:
                    raise TypeError("Restraint type not supported")

    # Releases

    # Point Loads

    # Element Loads

    def _show_grid_with_min_bounds(self, scale=0.5):
        # mesh bounds
        xmin, xmax, ymin, ymax, zmin, zmax = self.bounds
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
            color='#969594' 
        )
        self.plotter.show_grid(
            xtitle="X",
            ytitle="Y",
            ztitle="Z",
            font_size=12,
            location="outer",
            color='#969594'
        )

    def show(self):
        self.plotter.set_background('#1f1f1f')
        self.plotter.add_axes(
            color='#FAF9F6',
            x_color='#f96982',
            y_color='#96ef6d',
            z_color='#6ec8fa'
        )
        self._add_node_mesh() # keep here to correctly show bounds
        self._add_element_mesh()
        self._add_deformed_node_mesh()
        self._add_deformed_element_mesh()
        self._add_restraints_mesh()
        self._show_grid_with_min_bounds(scale=0.5)
        self.plotter.camera_position = [
            (1, 1, 1),   # auto-scaled after reset
            (0, 0, 0),
            (0, 1, 0),   # Y up
        ]
        self.plotter.reset_camera()
        self.plotter.enable_parallel_projection()
        self.plotter.disable_shadows()
        self.plotter.show()