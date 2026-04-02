from src.core.geometry.base_element import Element
from src.utils import global_variables as gv
import numpy as np

class Frame(Element):
    NODE_DOF_INDICES = [0, 1, 2, 3, 4, 5]
    LOCAL_DOFs_PER_NODE = ["ux", "uy", "uz", "rx", "ry", "rz"]
    LOCAL_FORCES_PER_NODE = ["Nx", "Vy", "Vz", "Tx", "My", "Mz"]

    GLOBAL_FORCES_PER_NODE = ["FX", "FY", "FZ", "MX", "MY", "MZ"] 
    ALL_DOFs = [ 
        (gv.NODE_i, gv.ux), (gv.NODE_i, gv.uy), (gv.NODE_i, gv.uz), 
        (gv.NODE_i, gv.rx), (gv.NODE_i, gv.ry), (gv.NODE_i, gv.rz),
        (gv.NODE_j, gv.ux), (gv.NODE_j, gv.uy), (gv.NODE_j, gv.uz),
        (gv.NODE_j, gv.rx), (gv.NODE_j, gv.ry), (gv.NODE_j, gv.rz)
    ]  
     
    def __init__(self, element_id, node_i, node_j, material, section, roll_radians = 0.0):    
        super().__init__(element_id, node_i, node_j, material, section, roll_radians)
        self.releases = set()
        
    def transformation_matrix(self): 
        R = self.rotation_matrix()
        T = np.zeros((12, 12))

        T[0:3, 0:3]   = R    
        T[3:6, 3:6]   = R    
        T[6:9, 6:9]   = R    
        T[9:12, 9:12] = R

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
                    
    def release(self, node:int, local_dof:int):
        """
        Release a DOF at a node.\n
        node (int): 0 for start node or 1 for end node.\n
        dof (int): local DOF index (0-5).
        """
        if node not in (0, 1):
            raise ValueError("Node must be 0(start) or 1(end)")
        
        self.releases.add((node, local_dof))

    def kept_and_released_indices(self) -> list:
        """ 
        kept     : list[tuple]  element-local DOFs kept in the system
        released : list[tuple]  element-local DOFs released
        """
        released = sorted(self.releases)
        kept = [dof for dof in self.ALL_DOFs if dof not in released]

        return kept, released
        
    def condensed_stiffness(self):
        """
        Local Element Stiffness Matrix where the released DOFs are eliminated.
        """
        k_local = self.local_stiffness()
        kept, released = self.kept_and_released_indices()

        # k_local can only use 1D vectors
        kept_vctr = [self.dofs_to_vctr_idx[(node, dof)] for node, dof in kept]
        released_vctr = [self.dofs_to_vctr_idx[(node, dof)] for node, dof in released]

        if not released:
            return k_local

        k_kk = k_local[np.ix_(kept_vctr, kept_vctr)]
        k_kr = k_local[np.ix_(kept_vctr, released_vctr)]
        k_rk = k_local[np.ix_(released_vctr, kept_vctr)]
        k_rr = k_local[np.ix_(released_vctr, released_vctr)]

        if k_rr.size == 0:
            return k_local

        k_cond = k_kk - k_kr @ np.linalg.inv(k_rr) @ k_rk
        return k_cond

    def full_condensed_stiffness(self):
        """
        A reconstructed nxn matrix where the values from the condensed stiffness 
        are placed back into their original positions (corresponding to kept DOFs),
        with zeros elsewhere.
        """
        k_local = self.local_stiffness()
        kept, released = self.kept_and_released_indices()
        
        if not released:
            return k_local
        
        k_cond = self.condensed_stiffness()
        k_cond_full = np.zeros_like(k_local)
        for a, i in enumerate(kept):
            for b, j in enumerate(kept):
                # k_cond_full[i, j] = k_cond[a, b]
                idx_i = self.dofs_to_vctr_idx[i]
                idx_j = self.dofs_to_vctr_idx[j]
                k_cond_full[idx_i, idx_j] = k_cond[a, b]

        return k_cond_full

    def global_stiffness(self):
        T = self.transformation_matrix()
        k_local_released = self.full_condensed_stiffness()
        return T.T @ k_local_released @ T
    
        