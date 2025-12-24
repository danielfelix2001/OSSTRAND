#source/model/lineElements/element.py

from abc import abstractmethod
from math import sqrt
from source.model.nodes import Node
from source.model.functions import local_dof_map
import numpy as np

ux, uy, uz = 0, 1, 2
rx, ry, rz = 3, 4, 5
NODE_i, NODE_j = 0, 1

class Element:
    DOFS_PER_NODE = [] # Element declares DOFs    
    NODE_DOF_INDICES = []
    
    def __init__(self, element_id: str, 
                 node_i: Node, node_j: Node, 
                 material, section, roll_radians: float = 0.0):
        # Element properties
        self.id = element_id
        self.i = node_i        # start node
        self.j = node_j        # end node
        self.material = material
        self.section = section
        self.roll = roll_radians 

        # Loads and reactions
        self.loads = []
        self.fef_local = None
        self.end_forces_local  = None
        self.end_forces_global = None

        # Helper properties
        self.dofs_to_vector_index = local_dof_map(self)

    @property
    def ndof_element(self):
        return len(self.NODE_DOF_INDICES) * 2

    # --------------------------------
    # GEOMETRY
    # --------------------------------
    #region
    def length(self) -> float:
        dx = self.j.x - self.i.x
        dy = self.j.y - self.i.y
        dz = self.j.z - self.i.z
        return sqrt(dx*dx + dy*dy + dz*dz)
    
    def local_axes(self):
        L = self.length()
        cos_phi = np.cos(self.roll)
        sin_phi = np.sin(self.roll)

        lx = (self.j.x - self.i.x)/L
        mx = (self.j.y - self.i.y)/L
        nx = (self.j.z - self.i.z)/L
        x_local = np.array([lx, mx, nx])

        ref_vector = np.array([0.0, 1.0, 0.0])
       
        # Can be made prettier using quaternions
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
    #endregion

    # ---------------------------------------------
    # TRANSFORMATION AND LOCAL STIFFNESS MATRICES
    # ---------------------------------------------
    #region
    @abstractmethod
    def transformation_matrix(self): #12x12
        pass
    
    @abstractmethod
    def local_stiffness(self):
        pass
    #endregion
    
    # --------------------------------
    # GLOBAL STIFFNESS MATRIX
    # --------------------------------
    #region
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
    #endregion 

    # --------------------------------
    # FIXED-END FORCES
    # --------------------------------
    #region
    def add_load(self, load):
        self.loads.append(load)

    def compute_fef(self):
        self.fef_local[:] = 0.0 
        for load in self.loads:
            self.fef_local += load.fef_local(self)
    #endregion
    
    # --------------------------------
    # LOCAL END FORCE ACCESSORS
    # -------------------------------- 
    #region
    # AXIAL 
    @property
    def Nx_i(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_i, ux)]]
    @property
    def Nx_j(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_j, ux)]]

    # SHEAR
    @property
    def Vy_i(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_i, uy)]]
    @property
    def Vz_i(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_i, uz)]]
    @property
    def Vy_j(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_j, uy)]]
    @property
    def Vz_j(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_j, uz)]]

    # BENDING
    @property
    def My_i(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_i, ry)]]
    @property
    def Mz_i(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_i, rz)]]
    @property
    def My_j(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_j, ry)]]
    @property
    def Mz_j(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_j, rz)]]

    # TORSION
    @property
    def Tx_i(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_i, rx)]]
    @property
    def Tx_j(self) -> float:
        return self.end_forces_local[self.dofs_to_vector_index[(NODE_j, rx)]]
    #endregion
    
    # --------------------------------
    # INTERNAL FORCE ACCESSORS
    # --------------------------------
    #region
    # AXIAL
    def Nx_internal(self, x) -> float:
        Nx_int = self.Nx_i
        Nx_load = 0.0

        for elementLoad in self.loads:
            Nx_load += elementLoad.axial(x, self)
        # compression is negative
        return -Nx_int - Nx_load    
    
    # SHEAR
    def Vy_internal(self, x) -> float:
        Vy_int = self.Vy_i
        Vy_load = 0.0

        for elementLoad in self.loads:
            Vy_load += elementLoad.shear_y(x, self)
        # clockwise inducing is positive
        return Vy_int + Vy_load 

    def Vz_internal(self, x) -> float:
        Vz_int = self.Vz_i
        Vz_load = 0.0

        for elementLoad in self.loads:
            Vz_load += elementLoad.shear_z(x, self)
        # clockwise inducing is positive
        return Vz_int + Vz_load 

    # BENDING    
    def My_internal(self, x) -> float:
        My_int = self.My_i
        My_load = 0.0

        for elementLoad in self.loads:
            My_load += elementLoad.moment_y(x, self)
        # frown inducing is negative
        return -My_int - My_load 

    def Mz_internal(self, x) -> float:
        Mz_int = self.Mz_i
        Mz_load = 0.0

        for elementLoad in self.loads:
            Mz_load += elementLoad.moment_z(x, self)
        # frown inducing is negative
        return -Mz_int - Mz_load 

    # TORSION
    def Tx_internal(self, x) -> float:
        Tx_int = self.Tx_i
        Tx_load = 0.0

        for elementLoad in self.loads:
            Tx_load += elementLoad.torsion(x, self)
        # against shaft axis is negative
        return -Tx_int - Tx_load
    #endregion
    