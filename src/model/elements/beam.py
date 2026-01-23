# src/model/elements/beam.py

from src.model.elements.frame import Frame
import numpy as np

class Beam(Frame):
    NODE_DOF_INDICES = [1, 2, 4, 5]
    LOCAL_DOFS_PER_NODE = ["uy", "uz", "ry", "rz"]
    LOCAL_FORCES_PER_NODE = ["Vy", "Vz", "My", "Mz"]  
      
    GLOBAL_FORCES_PER_NODE = ["FY", "FZ", "MY", "MZ"]    
    
    def __init__(self, element_id, node_i, node_j, material, section, roll_radians = 0.0):    
        super().__init__(element_id, node_i, node_j, material, section, roll_radians)
        self.fef_local = np.zeros(8) # fefs in local coordinates
        self.end_forces_local  = np.zeros(8)
        self.end_forces_global = np.zeros(8)

    def reset(self):
        self.loads = []
        self.fef_local = np.zeros(8)
        self.end_forces_local  = np.zeros(8)
        self.end_forces_global = np.zeros(8)

    def transformation_matrix(self): #8x8
        R = self.rotation_matrix()
        T = np.zeros((8, 8))

        # Node i
        T[0:2, 0:2] = R[1:3, 1:3]   # uy, uz
        T[2:4, 2:4] = R[1:3, 1:3]   # ry, rz

        # Node j
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
