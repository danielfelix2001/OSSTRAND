from src.core.geometry.base_element import Element
from src.utils import global_variables as gv
import numpy as np

class Truss(Element):
    NODE_DOF_INDICES = [0, 1, 2]
    LOCAL_DOFs_PER_NODE = ["ux", "uy", "uz"]
    LOCAL_FORCES_PER_NODE = ["Nx"]
    
    GLOBAL_FORCES_PER_NODE = ["FX", "FY", "FZ"]
    ALL_DOFs = [ 
        (gv.NODE_i, gv.ux), (gv.NODE_i, gv.uy), (gv.NODE_i, gv.uz), 
        (gv.NODE_j, gv.ux), (gv.NODE_j, gv.uy), (gv.NODE_j, gv.uz)
    ]       

    def __init__(self, element_id, node_i, node_j, material, section):    
        super().__init__(element_id, node_i, node_j, material, section)

    def transformation_matrix(self):
        x, _, _ = self.local_axes()
        l, m, n = x

        return np.array([
            [ l, m, n, 0, 0, 0],
            [ 0, 0, 0, l, m, n]
        ])

    def local_stiffness(self):
        E = self.material.E
        A = self.section.area
        L = self.length()
        k = E * A / L

        return np.array([[ k, -k],
                         [-k,  k]])
    
    def release(self, node:int, dof:int):
        raise NotImplementedError("Truss elements do not support DOF releases")
    
    def kept_and_released_indices(self) -> list:
        kept = self.ALL_DOFs
        released = []
        return kept, released
    
    def condensed_stiffness(self):
        return self.local_stiffness()
    
    def full_condensed_stiffness(self):
        return self.local_stiffness()
    
    def global_stiffness(self):
        T = self.transformation_matrix()
        k_local = self.local_stiffness()
        return T.T @ k_local @ T
     