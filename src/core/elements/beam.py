from src.core.elements.frame import Frame
from src.utils import global_variables as gv
import numpy as np

class Beam(Frame):
    NODE_DOF_INDICES = [1, 2, 4, 5]
    LOCAL_DOFs_PER_NODE = ["uy", "uz", "ry", "rz"]
    LOCAL_FORCES_PER_NODE = ["Vy", "Vz", "My", "Mz"]  
      
    GLOBAL_FORCES_PER_NODE = ["FY", "FZ", "MY", "MZ"]  
    ALL_DOFs = [ 
        (gv.NODE_i, gv.uy), (gv.NODE_i, gv.uz), 
        (gv.NODE_i, gv.ry), (gv.NODE_i, gv.rz),
        (gv.NODE_j, gv.uy), (gv.NODE_j, gv.uz),
        (gv.NODE_j, gv.ry), (gv.NODE_j, gv.rz)
    ]   
    
    def __init__(self, element_id, node_i, node_j, material, section, roll_radians = 0.0):    
        super().__init__(element_id, node_i, node_j, material, section, roll_radians)
        self.releases = set()

    def transformation_matrix(self): #8x8
        R = self.rotation_matrix()
        T = np.zeros((8, 8))

        T[0:2, 0:2] = R[1:3, 1:3]   # extracted y-z rotation block from frame element
        T[2:4, 2:4] = R[1:3, 1:3]   
        T[4:6, 4:6] = R[1:3, 1:3]
        T[6:8, 6:8] = R[1:3, 1:3]

        return T

    def local_stiffness(self):
        k = super().local_stiffness()

        # Remove axial and torsion DOFs
        remove = [0, 3, 6, 9]  # ux_i, rx_i, ux_j, rx_j
        k = np.delete(k, remove, axis=0)
        k = np.delete(k, remove, axis=1)
        return k
    