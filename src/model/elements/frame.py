# src/model/elements/frame.py

from src.model.geometry.base_element import Element
import numpy as np

class Frame(Element):
    NODE_DOF_INDICES = [0, 1, 2, 3, 4, 5]
    LOCAL_DOFS_PER_NODE = ["ux", "uy", "uz", "rx", "ry", "rz"]
    LOCAL_FORCES_PER_NODE = ["Nx", "Vy", "Vz", "Tx", "My", "Mz"]

    GLOBAL_FORCES_PER_NODE = ["FX", "FY", "FZ", "MX", "MY", "MZ"]   
     
    def __init__(self, element_id, node_i, node_j, material, section, roll_radians = 0.0):    
        super().__init__(element_id, node_i, node_j, material, section, roll_radians)
        self.fef_local = np.zeros(12) # fefs in local coordinates
        self.end_forces_local  = np.zeros(12)
        self.end_forces_global = np.zeros(12)

        self.releases = {
            "i": set(),
            "j": set()
        }
        
    def reset(self):
        self.loads = []
        self.fef_local = np.zeros(12)
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
            
    def release(self, node:str, dof):
        """
        Release a DOF at a node.\n
        node (str): 'i' for start node or 'j' for end node.\n
        dof (int): local DOF index (0-5).
        """
        if dof not in self.NODE_DOF_INDICES:
            raise ValueError(f"Invalid DOF for Element: {dof}")
        
        if node == "j":
            dof+=6    
        elif node != "i":
            raise ValueError(f"Invalid node: select 'i' or 'j'")
        
        self.releases[node].add(dof)

    def apply_releases(self, k_local):
        released = set(self.releases["i"]) | set(self.releases["j"])
        if not released:
            return k_local

        kept = [i for i in range(12) if i not in released]

        k_kk = k_local[np.ix_(kept, kept)]
        k_kr = k_local[np.ix_(kept, list(released))]
        k_rk = k_local[np.ix_(list(released), kept)]
        k_rr = k_local[np.ix_(list(released), list(released))]

        if k_rr.size == 0:
            return k_local

        k_cond = k_kk - k_kr @ np.linalg.inv(k_rr) @ k_rk

        k_full = np.zeros((12, 12))
        for a, i in enumerate(kept):
            for b, j in enumerate(kept):
                k_full[i, j] = k_cond[a, b]

        return k_full

    def global_stiffness(self):
        T = self.transformation_matrix()
        k_local = self.local_stiffness()
        k_local = self.apply_releases(k_local)
        return T.T @ k_local @ T
    
    # --------------------------------
    # FIXED-END FORCES
    # --------------------------------
    def add_load(self, load):
        self.loads.append(load)
        
    def compute_fef(self):
        self.fef_local[:] = 0.0 
        for load in self.loads:
            self.fef_local += load.fef_local(self)

        # Remove fefs on release dofs
        NODE_i = 0
        NODE_j = 1
        for dof in self.releases["i"]:
            self.fef_local[self.dofs_to_vector_index[NODE_i, dof]] = 0.0
        for dof in self.releases["j"]:
            self.fef_local[self.dofs_to_vector_index[NODE_j, dof]] = 0.0
        