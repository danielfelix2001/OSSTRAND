#src/model/elements.py

from math import sqrt
from src.model.nodes import Node
import numpy as np

class Element:
    def __init__(self, element_id: str, 
                 node_i: Node, node_j: Node, 
                 material, section, roll_radians: float = 0.0):
        self.id = element_id
        self.i = node_i        # start node
        self.j = node_j        # end node
        self.material = material
        self.section = section
        self.roll = roll_radians 
        self.fef_local = np.zeros(12) # fefs in local coordinates

    def length(self) -> float:
        dx = self.j.x - self.i.x
        dy = self.j.y - self.i.y
        dz = self.j.z - self.i.z
        return sqrt(dx*dx + dy*dy + dz*dz)
    
    # --------------------------------
    # FIXED-END FORCES
    # --------------------------------
    # fixed-end forces are REACTIONS to the non-nodal loads

    def add_udl(self, qy: float = 0.0, qz: float = 0.0):
        L = self.length()
        
        # ---- UDL in local y direction ----
        if qy != 0.0:
            f = qy * L / 2
            m = qy * L**2 / 12

            self.fef_local[1] -= f      # Fy_i
            self.fef_local[5] -= m      # Mz_i

            self.fef_local[7] -= f      # Fy_j
            self.fef_local[11] += m     # Mz_j

        # ---- UDL in local z direction ----
        if qz != 0.0:
            f = qz * L / 2
            m = qz * L**2 / 12

            self.fef_local[2] -= f      # Fz_i
            self.fef_local[4] += m      # My_i

            self.fef_local[8] -= f      # Fz_j
            self.fef_local[10] -= m     # My_j

    # --------------------------------
    # TRANSFORMATION MATRIX
    # --------------------------------
    def local_axes(self):
        xi, yi, zi = self.i.x, self.i.y, self.i.z
        xj, yj, zj = self.j.x, self.j.y, self.j.z

        dx = xj - xi
        dy = yj - yi
        dz = zj - zi   

        L = self.length()
        x_local = np.array([dx/L, dy/L, dz/L])

        global_up = np.array([0.0, 1.0, 0.0])

        if np.isclose(abs(np.dot(x_local, global_up)), 1.0):
            global_up = np.array([0.0, 0.0, 1.0])  

        # Include angle of roll about local x axis
        y0 = global_up - np.dot(global_up, x_local) * x_local
        y0 /= np.linalg.norm(y0)
        z0 = np.cross(x_local, y0)        

        phi = self.roll
        def rotate_about_local_x(v, axis, angle_rad):
            cos_a = np.cos(angle_rad)
            sin_a = np.sin(angle_rad)
            return (v * cos_a + 
                    np.cross(axis, v) * sin_a + 
                    axis * np.dot(axis, v) * (1 - cos_a))

        y_local = rotate_about_local_x(y0, x_local, phi)
        z_local = rotate_about_local_x(z0, x_local, phi)

        return x_local, y_local, z_local

    def rotation_matrix(self): #3x3
        x, y, z = self.local_axes()
        return np.vstack([x, y, z])

    def transformation_matrix(self): #12x12
        R = self.rotation_matrix()
        T = np.zeros((12, 12))

        for i in range(4):
            T[i*3:(i+1)*3, i*3:(i+1)*3] = R

        return T

    # --------------------------------
    # STIFFNESS MATRICES
    # --------------------------------
    def local_stiffness(self):
        E = self.material.E
        G = self.material.G
        A = self.section.area
        J = self.section.J
        L = self.length()

        # If y is up,       
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

    def global_stiffness(self):
        T = self.transformation_matrix()
        k_local = self.local_stiffness()
        k_global = T.T @ k_local @ T
        return k_global
    
    def get_dof_indices(self):
        dofs = []
        for node in [self.i, self.j]:
            dofs.extend(node.dof)
        return dofs    

    def __repr__(self):
        return (
            f"Element(id={self.id}, "
            f"node_i={self.i.id}, node_j={self.j.id}, "
            f"length={self.length():.3f})"
        )
