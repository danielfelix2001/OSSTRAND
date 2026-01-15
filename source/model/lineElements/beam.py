#source/model/lineElements/beam.py

from source.model.lineElements.frame import Frame
import numpy as np

class Beam(Frame):
    NODE_DOF_INDICES = [1, 2, 4, 5]
    LOCAL_DOFS_PER_NODE = ["uy", "uz", "ry", "rz"]
    LOCAL_FORCES_PER_NODE = ["Vy", "Vz", "My", "Mz"]  
      
    GLOBAL_FORCES_PER_NODE = ["VY", "VZ", "MY", "MZ"]    
    
    def __init__(self, element_id, node_i, node_j, material, section, roll_radians = 0.0):    
        super().__init__(element_id, node_i, node_j, material, section, roll_radians)
        self.fef_local = np.zeros(8) # fefs in local coordinates
        self.end_forces_local  = np.zeros(8)
        self.end_forces_global = np.zeros(8)
    
    def transformation_matrix(self): #8x8
        R = self.rotation_matrix()
        T = np.zeros((8, 8))

        # Only y and z translations and rotations
        for i in range(4):
            T[i*2:(i+1)*2, i*2:(i+1)*2] = R[1:3, 1:3]

        return T

    def local_stiffness(self):
        k = super().local_stiffness()

        # Remove axial and torsion DOFs
        remove = [0, 3, 6, 9]  # ux_i, rx_i, ux_j, rx_j
        k = np.delete(k, remove, axis=0)
        k = np.delete(k, remove, axis=1)
        return k
