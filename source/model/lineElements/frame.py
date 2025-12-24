#source/model/lineElements/frame.py

from source.model.lineElements.element import Element
import numpy as np

class Frame(Element):
    DOFS_PER_NODE = ["ux", "uy", "uz", "rx", "ry", "rz"]
    NODE_DOF_INDICES = [0, 1, 2, 3, 4, 5]

    def __init__(self, element_id, node_i, node_j, material, section, roll_radians = 0.0):    
        super().__init__(element_id, node_i, node_j, material, section, roll_radians)
        self.fef_local = np.zeros(12) # fefs in local coordinates
        self.end_forces_local  = np.zeros(12)
        self.end_forces_global = np.zeros(12)

    def transformation_matrix(self): #12x12
        R = self.rotation_matrix()
        T = np.zeros((12, 12))

        for i in range(4):
            T[i*3:(i+1)*3, i*3:(i+1)*3] = R

        return T
    
    def local_stiffness(self):
        E = self.material.E
        G = self.material.G
        A = self.section.area
        J = self.section.J
        L = self.length()

        Iy = self.section.Iyy # weak axis is bending about y
        Iz = self.section.Ixx # strong axis is bending about z

        k = np.zeros((12, 12))

        # axial
        k[0, 0] = k[6, 6] =  E*A / L
        k[0, 6] = k[6, 0] = -E*A / L

        # torsion
        k[3, 3] = k[9, 9] =  G*J / L
        k[3, 9] = k[9, 3] = -G*J / L

        # bending about local z
        k[1, 1]  = k[7, 7]  =  12*E*Iz / L**3
        k[1, 7]  = k[7, 1]  = -12*E*Iz / L**3

        k[1, 5]  = k[5, 1]  =  6*E*Iz / L**2
        k[1,11]  = k[11,1]  =  6*E*Iz / L**2
        k[5, 7]  = k[7, 5]  = -6*E*Iz / L**2        
        k[7, 11] = k[11, 7] = -6*E*Iz / L**2        

        k[5, 5]  = k[11,11] =  4*E*Iz / L
        k[5,11]  = k[11,5]  =  2*E*Iz / L

        # bending about local y
        k[2, 2]  = k[8, 8]  =  12*E*Iy / L**3
        k[2, 8]  = k[8, 2]  = -12*E*Iy / L**3

        k[2, 4]  = k[4, 2]  = -6*E*Iy / L**2
        k[2,10]  = k[10,2]  = -6*E*Iy / L**2
        k[4, 8]  = k[8, 4]  =  6*E*Iy / L**2
        k[8,10]  = k[10,8]  =  6*E*Iy / L**2

        k[4, 4]  = k[10,10] =  4*E*Iy / L
        k[4,10]  = k[10,4]  =  2*E*Iy / L

        return k     
  