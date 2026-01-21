#source/model/fixedEndForces/fefs.py

from abc import ABC, abstractmethod
import numpy as np

ux, uy, uz = 0, 1, 2
rx, ry, rz = 3, 4, 5
NODE_i, NODE_j = 0, 1

class ElementLoad(ABC):
    @abstractmethod
    def fef_local(self, element):
        """
        Returns local fixed-end force vector in local coordinates.
        Size depends on element type
        """
        pass

    # Analytical Internal Force Contributions
    # Will be called in Element class to compute internal forces
    # The formula for internal force is:
    # F_int(x) = F_NODE_i + ∑F_LOAD(x)
    # Applied Local End Forces is F_NODE_i
    # shear_y, moment_z, etc. are ∑F_LOAD(x)

    # Local y 
    def shear_y(self, x, element):
        return 0.0
    def moment_z(self, x, element):
        return 0.0

    # Local z
    def shear_z(self, x, element):
        return 0.0
    def moment_y(self, x, element):
        return 0.0

    # Local x
    def axial(self, x, element):
        return 0.0
    def torsion(self, x, element):
        return 0.0
    
class UniformlyDistributedLoad(ElementLoad):
    def __init__(self, local, wx=0.0, wy=0.0, wz=0.0):     
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        self.isLocal = local

        # Containers
        self.wxInput = wx
        self.wyInput = wy
        self.wzInput = wz

    def fef_local(self, element):
        L = element.length()
        nd = element.numberOfDOFs
        fefs = np.zeros(nd)
        dofs_to_idx = element.dofs_to_vector_index

        if self.isLocal:
            self.wx = self.wxInput
            self.wy = self.wyInput
            self.wz = self.wzInput

        else:
            x_global = np.array([1.0, 0.0, 0.0])
            y_global = np.array([0.0, 1.0, 0.0])
            z_global = np.array([0.0, 0.0, 1.0])
        
            x_local, y_local, z_local = element.local_axes()

            # Decompose Global UDLs to local axes
            # Local x
            self.wx = (self.wxInput * np.dot(x_global, x_local) +
                       self.wyInput * np.dot(y_global, x_local) +
                       self.wzInput * np.dot(z_global, x_local))
            # Local y
            self.wy = (self.wxInput * np.dot(x_global, y_local) +
                       self.wyInput * np.dot(y_global, y_local) +
                       self.wzInput * np.dot(z_global, y_local))
            # Local z
            self.wz = (self.wxInput * np.dot(x_global, z_local) +
                       self.wyInput * np.dot(y_global, z_local) +
                       self.wzInput * np.dot(z_global, z_local))            

        # ---- Local x load ----
        if (NODE_i, ux) in dofs_to_idx and self.wx != 0.0:    # checks just to be sure
            f = self.wx * L / 2

            fefs[dofs_to_idx[(NODE_i, ux)]] -= f
            fefs[dofs_to_idx[(NODE_j, ux)]] -= f

        # ---- Local y load → bending about z ----
        if self.wy != 0.0:
            f = self.wy * L / 2
            m = self.wy * L**2 / 12

            fefs[dofs_to_idx[(NODE_i, uy)]]  -= f     # dof[(node, dof_type)]
            fefs[dofs_to_idx[(NODE_i, rz)]]  -= m     # node i = 0
            fefs[dofs_to_idx[(NODE_j, uy)]]  -= f     # node j = 1
            fefs[dofs_to_idx[(NODE_j, rz)]]  += m

        # ---- Local z load → bending about y ----
        if self.wz != 0.0:
            f = self.wz * L / 2
            m = self.wz * L**2 / 12

            fefs[dofs_to_idx[(NODE_i, uz)]] -= f
            fefs[dofs_to_idx[(NODE_i, ry)]] += m
            fefs[dofs_to_idx[(NODE_j, uz)]] -= f
            fefs[dofs_to_idx[(NODE_j, ry)]] -= m

        return fefs
    
    # Local x
    def axial(self, x, element):
        return self.wx * x

    # Local y 
    def shear_y(self, x, element):
        return self.wy * x
    def moment_z(self, x, element):
        return 0.5 * self.wy * x**2

    # Local z
    def shear_z(self, x, element):
        return self.wz * x
    def moment_y(self, x, element):
        return 0.5 * self.wz * x**2
        
class SelfWeight(ElementLoad):
    def __init__(self, loadFactor):
        self.loadFactor = loadFactor
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0

    def fef_local(self, element):
        L = element.length()
        nd = element.numberOfDOFs
        fefs = np.zeros(nd)
        dofs_to_idx = element.dofs_to_vector_index

        gravity_vector = np.array([0.0, -1.0, 0.0])
        x_local, y_local, z_local = element.local_axes()
        selfWeight = element.material.gamma * element.section.area * self.loadFactor

        # Decompose self weight to local x,y,z
        self.wx = selfWeight * np.dot(gravity_vector, x_local)
        self.wy = selfWeight * np.dot(gravity_vector, y_local)
        self.wz = selfWeight * np.dot(gravity_vector, z_local)
        
        # ---- Local y load bending about z ----
        if self.wy != 0.0:
            f = self.wy * L / 2
            m = self.wy * L**2 / 12

            fefs[dofs_to_idx[(NODE_i, uy)]]  -= f     # dof[(node, dof_type)]
            fefs[dofs_to_idx[(NODE_i, rz)]]  -= m     # node i = 0
            fefs[dofs_to_idx[(NODE_j, uy)]]  -= f     # node j = 1
            fefs[dofs_to_idx[(NODE_j, rz)]]  += m

        # ---- Local z load bending about y ----
        if self.wz != 0.0:
            f = self.wz * L / 2
            m = self.wz * L**2 / 12

            fefs[dofs_to_idx[(NODE_i, uz)]] -= f
            fefs[dofs_to_idx[(NODE_i, ry)]] += m
            fefs[dofs_to_idx[(NODE_j, uz)]] -= f
            fefs[dofs_to_idx[(NODE_j, ry)]] -= m

        # ---- Local x load ----
        if (NODE_i, ux) in dofs_to_idx and self.wx != 0.0:    # checks if NODE_i has UX dof (for beams)
            f = self.wx * L / 2

            fefs[dofs_to_idx[(NODE_i, ux)]] -= f
            fefs[dofs_to_idx[(NODE_j, ux)]] -= f

        return fefs
    
    # Local y 
    def shear_y(self, x, element):
        return self.wy * x
    def moment_z(self, x, element):
        return 0.5 * self.wy * x**2

    # Local z
    def shear_z(self, x, element):
        return self.wz * x
    def moment_y(self, x, element):
        return 0.5 * self.wz * x**2
    
    # Local x
    def axial(self, x, element):
        return self.wx * x
     
class PointLoad(ElementLoad):
    def __init__(self, NODE_i_DISTANCE, local, px=0.0, py=0.0, pz=0.0):
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0
        self.a  = NODE_i_DISTANCE
        self.isLocal = local

        # Containers
        self.pxInput = px
        self.pyInput = py
        self.pzInput = pz

    def fef_local(self, element):
        L = element.length()
        a = self.a
        b = L-a
        nd = element.numberOfDOFs
        fefs = np.zeros(nd)
        dofs_to_idx = element.dofs_to_vector_index

        if self.isLocal:
            self.px = self.pxInput
            self.py = self.pyInput
            self.pz = self.pzInput
            
        else:
            x_global = np.array([1.0, 0.0, 0.0])
            y_global = np.array([0.0, 1.0, 0.0])
            z_global = np.array([0.0, 0.0, 1.0])
        
            x_local, y_local, z_local = element.local_axes()

            # Decompose Global point loads to local axes
            # Local x
            self.px = (self.pxInput * np.dot(x_global, x_local) +
                       self.pyInput * np.dot(y_global, x_local) +
                       self.pzInput * np.dot(z_global, x_local))
            # Local y
            self.py = (self.pxInput * np.dot(x_global, y_local) +
                       self.pyInput * np.dot(y_global, y_local) +
                       self.pzInput * np.dot(z_global, y_local))
            # Local z
            self.pz = (self.pxInput * np.dot(x_global, z_local) +
                       self.pyInput * np.dot(y_global, z_local) +
                       self.pzInput * np.dot(z_global, z_local))             

        if (NODE_i, ux) in dofs_to_idx and self.px != 0.0:    # checks just to be sure
            f_i = self.px * (L-self.a)/L
            f_j = self.px * self.a/L

            fefs[dofs_to_idx[(NODE_i, ux)]] -= f_i
            fefs[dofs_to_idx[(NODE_j, ux)]] -= f_j

        if self.py != 0.0:
            f_i = self.py * b**2 * (3*a + b) / L**3
            m_i = self.py * a * b**2 / L**2
            f_j = self.py * a**2 * (3*b + a) / L**3
            m_j = self.py * b * a**2 / L**2

            fefs[dofs_to_idx[(NODE_i, uy)]] -= f_i
            fefs[dofs_to_idx[(NODE_i, rz)]] -= m_i 
            fefs[dofs_to_idx[(NODE_j, uy)]] -= f_j
            fefs[dofs_to_idx[(NODE_j, rz)]] += m_j

        if self.pz != 0.0:
            f_i = self.pz * b**2 * (3*a + b) / L**3
            m_i = self.pz * a * b**2 / L**2
            f_j = self.pz * a**2 * (3*b + a) / L**3
            m_j = self.pz * b * a**2 / L**2

            fefs[dofs_to_idx[(NODE_i, uy)]] -= f_i
            fefs[dofs_to_idx[(NODE_i, rz)]] += m_i 
            fefs[dofs_to_idx[(NODE_j, uy)]] -= f_j
            fefs[dofs_to_idx[(NODE_j, rz)]] -= m_j

        return fefs
    
    # Local y
    def shear_y(self, x, element):
        Vy = 0.0
        # discontinuous at x = a
        if x < self.a:
            Vy = 0.0
        if x > self.a:
            Vy = self.py
        return Vy
    
    def moment_z(self, x, element):
        Mz = 0.0
        # discontinuous at x = a
        if x < self.a:
            Mz = 0
        if x > self.a:
            Mz = self.py * (x-self.a)
        return Mz

    # Local z    
    def shear_z(self, x, element):
        Vz = 0.0
        # discontinuous at x = a
        if x < self.a:
            Vz = 0.0
        if x > self.a:
            Vz = self.pz
        return Vz
    
    def moment_y(self, x, element):
        My = 0.0
        # discontinuous at x = a
        if x < self.a:
            My = 0
        if x > self.a:
            My = self.pz * (x-self.a)
        return My
    
    # Local x
    def axial(self, x, element):
        Nx = 0.0
        # discontinuous at x = a
        if x < self.a:
            Nx = 0.0
        if x > self.a:
            Nx = self.px
        return Nx

class PolynomialLoad(ElementLoad):
    # use integration to compute fef
    pass