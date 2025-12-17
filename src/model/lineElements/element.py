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

    def required_dofs(self):
        return self.DOFS_PER_NODE

    # --------------------------------
    # GEOMETRY
    # --------------------------------
    def length(self) -> float:
        dx = self.j.x - self.i.x
        dy = self.j.y - self.i.y
        dz = self.j.z - self.i.z
        return sqrt(dx*dx + dy*dy + dz*dz)
    
    def local_axes(self):
        dx = self.j.x - self.i.x
        dy = self.j.y - self.i.y
        dz = self.j.z - self.i.z  

        L = self.length()
        x_local = np.array([dx/L, dy/L, dz/L]) #along element length

        global_up = np.array([0.0, 1.0, 0.0])
        if np.isclose(abs(np.dot(x_local, global_up)), 1.0):
            global_up = np.array([0.0, 0.0, 1.0])  

        # Include angle of roll about local x axis
        y0 = global_up - np.dot(global_up, x_local) * x_local
        y0 /= np.linalg.norm(y0)
        z0 = np.cross(x_local, y0)        

        phi = self.roll
        def rotate_about_local_x(v, axis, angle_rad):
            cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
            return (v * cos_a + 
                    np.cross(axis, v) * sin_a + 
                    axis * np.dot(axis, v) * (1 - cos_a))

        y_local = rotate_about_local_x(y0, x_local, phi)
        z_local = rotate_about_local_x(z0, x_local, phi)

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

    def get_dof_indices(self):
        dofs = []
        for node in (self.i, self.j):
            for idx in self.NODE_DOF_INDICES:
                dofs.append(node.dofs[idx])
        return dofs  
    
    # --------------------------------
    # GLOBAL STIFFNESS MATRIX
    # --------------------------------
    def global_stiffness(self):
        T = self.transformation_matrix()
        k_local = self.local_stiffness()
        k_global = T.T @ k_local @ T
        return k_global
    
    def __repr__(self):
        return (
            f"Element(id={self.id}, "
            f"node_i={self.i.id}, node_j={self.j.id}, "
            f"length={self.length():.3f})"
        )
