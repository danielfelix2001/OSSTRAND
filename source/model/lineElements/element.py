#source/model/lineElements/element.py

from abc import abstractmethod
from math import sqrt
from source.model.nodes import Node
from source.model.functions import local_dof_map
from source.model.exceptions import ElementError, ModelDefinitionError
import numpy as np

ux, uy, uz = 0, 1, 2
rx, ry, rz = 3, 4, 5
NODE_i, NODE_j = 0, 1

class Element:
    NODE_DOF_INDICES = []
    LOCAL_DOFS_PER_NODE = [] # Element declares DOFs    
    LOCAL_FORCES_PER_NODE = []    

    GLOBAL_FORCES_PER_NODE = []    
    
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

        # Error handling
        if self.material is None:
            raise ModelDefinitionError(
                f"Element {self.id} has no material assigned."
            )
        
        if self.section is None:
            raise ModelDefinitionError(
                f"Element {self.id} has no section assigned."
            )

    @property
    def numberOfDOFs(self):
        return len(self.NODE_DOF_INDICES) * 2

    # --------------------------------
    # GEOMETRY
    # --------------------------------
    #region
    def length(self) -> float:
        dx = self.j.x - self.i.x
        dy = self.j.y - self.i.y
        dz = self.j.z - self.i.z
        L = sqrt(dx*dx + dy*dy + dz*dz)
        if L <= 0.0:
            raise ElementError(
                f"Element {self.id} has zero or negative length."
            )
        return L
    
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
    # TRANSFORMATION AND STIFFNESS MATRICES
    # ---------------------------------------------
    @abstractmethod
    def transformation_matrix(self):
        pass
    
    @abstractmethod
    def local_stiffness(self):
        pass
    
    @abstractmethod
    def global_stiffness(self):
        pass

    def get_dof_indices(self):
        dofs = []
        for node in (self.i, self.j):
            for idx in self.NODE_DOF_INDICES:
                dofs.append(node.dofs[idx]) # only add DOFs that the element asks for
        return dofs 

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
        Nx_NODE_i = self.Nx_i
        Nx_LOAD = 0.0
        for elementLoad in self.loads:
            Nx_LOAD += elementLoad.axial(x, self)
        return -(Nx_NODE_i + Nx_LOAD)    
    
    # SHEAR
    def Vy_internal(self, x) -> float:
        Vy_NODE_i = self.Vy_i
        Vy_LOAD = 0.0
        for elementLoad in self.loads:
            Vy_LOAD += elementLoad.shear_y(x, self)
        return Vy_NODE_i + Vy_LOAD 

    def Vz_internal(self, x) -> float:
        Vz_NODE_i = self.Vz_i
        Vz_LOAD = 0.0
        for elementLoad in self.loads:
            Vz_LOAD += elementLoad.shear_z(x, self)
        return Vz_NODE_i + Vz_LOAD 

    # BENDING    
    def My_internal(self, x) -> float:
        My_NODE_i = self.My_i + self.Vz_i * x
        My_LOAD = 0.0
        for elementLoad in self.loads:
            My_LOAD += elementLoad.moment_y(x, self)
        return My_NODE_i + My_LOAD 

    def Mz_internal(self, x) -> float:
        Mz_NODE_i = self.Mz_i - self.Vy_i * x
        Mz_LOAD = 0.0
        for elementLoad in self.loads:
            Mz_LOAD += elementLoad.moment_z(x, self)
        return -(Mz_NODE_i + Mz_LOAD) 

    # TORSION
    def Tx_internal(self, x) -> float:
        Tx_NODE_i = self.Tx_i
        Tx_LOAD = 0.0
        for elementLoad in self.loads:
            Tx_LOAD += elementLoad.torsion(x, self)
        return -(Tx_NODE_i + Tx_LOAD)
    #endregion
    
    # --------------------------------
    # INTERNAL STRESS ACCESSORS
    # --------------------------------
    #region

    # AXIAL 
    # AXIAL STRESS P/A
    def axial_stress(self, x) -> float:
        # positive if tension, negative if compression
        return self.Nx_internal(x) / self.section.area
    
    # BENDING STRESS Mc/I
    def bending_stress_about_y(self, x, c_z) -> float:
        """
        Returns the bending stress about the y-axis
        
        :param x: Length along the member
        :param c_z: Distance from the neutral axis
        :return: Bending Stress taken about local y-axis
        :rtype: float
        """
        # if positive bending, +z is in compression, should return negative
        return self.My_internal(x) * (-c_z)/self.section.Iyy 
    
    def bending_stress_about_z(self, x, c_y) -> float:
        """
        Returns the bending stress about the z-axis
        
        :param x: Length along the member
        :param c_y: Distance from the neutral axis
        :return: Bending Stress taken about local z-axis
        :rtype: float
        """
        # if positive bending, +y is in compression, should return negative
        return self.Mz_internal(x)  * (-c_y)/self.section.Ixx 

    def normal_stress(self, x, y, z):
        F_a   = self.axial_stress(x)
        F_b_y = self.bending_stress_about_y(x, z)
        F_b_z = self.bending_stress_about_z(x, y)

        return F_a + F_b_y + F_b_z
    # SHEAR
    # SIMPLE SHEAR STRESS V/A
    def simple_shear_stress_along_y(self, x) -> float:
        return self.Vy_internal(x) / self.section.area 
    # BENDING SHEAR STRESS VQ/Ib
    # TORSIONAL SHEAR STRESS Tr/J
    #endregion

    # --------------------------------
    # QUERYING API
    # --------------------------------
    def END_FORCES(self, node_label:str, local:bool):
        if local:
            forcesPerNode = self.LOCAL_FORCES_PER_NODE
            endForces = self.end_forces_local
        else:
            forcesPerNode = self.GLOBAL_FORCES_PER_NODE
            endForces = self.end_forces_global
        
        endForceDict = {}
        n = len(forcesPerNode)

        if node_label == "i":
            i = 0
        elif node_label == "j":
            i = n
        else:
            raise ValueError("Invalid node type: select \"i\" or \"j\"")
        
        for force in forcesPerNode:
            endForceDict[force] = endForces[i]
            i+=1
        
        return endForceDict
