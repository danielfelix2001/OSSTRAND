#src/model/lineElements/element.py

from abc import abstractmethod
from math import sqrt
from src.model.nodes import Node
import numpy as np

class Element:
    DOFS_PER_NODE = [] # Element declares DOFs    
    NODE_DOF_INDICES = []
    
    def __init__(self, element_id: str, 
                 node_i: Node, node_j: Node, 
                 material, section, roll_radians: float = 0.0):
        self.id = element_id
        self.i = node_i        # start node
        self.j = node_j        # end node
        self.material = material
        self.section = section
        self.roll = roll_radians 
        self.loads = []
        self.fef_local = None

    @property
    def ndof_element(self):
        return len(self.NODE_DOF_INDICES) * 2
    
    def local_dof_map(self):
        """
        Returns:
            {(node, dof_type): index_in_local_vector}
        """
        dof_map = {}
        idx = 0
        for node in (0, 1):  # 0 = i, 1 = j
            for dof in self.NODE_DOF_INDICES:
                dof_map[(node, dof)] = idx
                idx += 1
        return dof_map

    # --------------------------------
    # FIXED-END FORCES
    # --------------------------------
    def add_load(self, load):
        self.loads.append(load)

    def compute_fef(self):
        self.fef_local[:] = 0.0     # test first
        for load in self.loads:
            self.fef_local += load.fef_local(self)

    # --------------------------------
    # GEOMETRY
    # --------------------------------
    def length(self) -> float:
        dx = self.j.x - self.i.x
        dy = self.j.y - self.i.y
        dz = self.j.z - self.i.z
        return sqrt(dx*dx + dy*dy + dz*dz)
    
    def local_axes(self): # direction cosines
        L = self.length()
        cos_phi = np.cos(self.roll)
        sin_phi = np.sin(self.roll)

        lx = (self.j.x - self.i.x)/L
        mx = (self.j.y - self.i.y)/L
        nx = (self.j.z - self.i.z)/L
        x_local = np.array([lx, mx, nx])

        ref_vector = np.array([0.0, 1.0, 0.0])
       
        if abs(np.dot(x_local, ref_vector))> 0.9:  # if element is almost vertical
            denominator = sqrt(lx*lx + mx*mx)
            y_local =  np.array([
                (- mx * cos_phi - lx * nx * sin_phi) / denominator,
                (  lx * cos_phi -  mx * nx * sin_phi) / denominator,
                denominator * sin_phi
            ])
            z_local =  np.array([
                ( mx * sin_phi - lx * nx * cos_phi) / denominator,
                (- lx * sin_phi - mx * nx * cos_phi) / denominator,
                denominator * cos_phi
            ])
        else: # if non-vertical
            denominator = sqrt(lx*lx + nx*nx)
            y_local = np.array([
                (- lx * mx *cos_phi - nx * sin_phi) /denominator,
                denominator * cos_phi,
                (- mx * nx * cos_phi +  lx * sin_phi) /denominator
            ])
            z_local = np.array([
                (lx * mx * sin_phi - nx * cos_phi) /denominator,
                - denominator * sin_phi,
                (mx * nx * sin_phi + lx * cos_phi) /denominator
            ])

        return x_local, y_local, z_local

    def rotation_matrix(self): #3x3
        x, y, z = self.local_axes()
        return np.vstack([x, y, z])

    # --------------------------------
    # ABSTRACT METHODS
    # IMPLEMENTED IN CHILD CLASSES
    # --------------------------------
    @abstractmethod
    def transformation_matrix(self): #12x12
        pass
    
    @abstractmethod
    def local_stiffness(self):
        pass
    
    # --------------------------------
    # GLOBAL STIFFNESS MATRIX
    # --------------------------------
    def global_stiffness(self):
        T = self.transformation_matrix()
        k_local = self.local_stiffness()
        return T.T @ k_local @ T

    def get_dof_indices(self):
        dofs = []
        for node in (self.i, self.j):
            for idx in self.NODE_DOF_INDICES:
                dofs.append(node.dofs[idx])
        return dofs  

