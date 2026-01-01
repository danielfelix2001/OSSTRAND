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

    # Analytical Internal Force increments
    # Will be called in Element class to compute internal forces

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
    
class UDL(ElementLoad):
    def __init__(self, wy=0.0, wz=0.0):     # local axes only
        self.wy = wy
        self.wz = wz

    def fef_local(self, element):
        L = element.length()
        nd = element.numberOfDOFs
        fefs = np.zeros(nd)
        dofs_to_idx = element.dofs_to_vector_index

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
    def __init__(self):
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
        selfWeight = element.material.gamma * element.section.area

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
        if (NODE_i, ux) and self.wx != 0.0:    # checks if NODE_i has UX dof (for beams)
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
 
class AxialUDL(ElementLoad):
    def __init__(self, wx = 0.0):
        self.wx = wx
    
    def fef_local(self, element):
        L = element.length()
        nd = element.numberOfDOFs
        fefs = np.zeros(nd)
        dofs_to_idx = element.dofs_to_vector_index

        if (NODE_i, ux) and self.wx != 0.0:    # checks just to be sure
            f = self.wx * L / 2

            fefs[dofs_to_idx[(NODE_i, ux)]] -= f
            fefs[dofs_to_idx[(NODE_j, ux)]] -= f

        return fefs
    
    # Local x
    def axial(self, x, element):
        return self.wx * x
    
class PointLoad(ElementLoad):
    def __init__(self, a, px = 0.0, py = 0.0, pz = 0.0):   # local axes only
        self.px = px
        self.py = py
        self.pz = pz
        self.a  = a # distance from NODE_i
    
    def fef_local(self, element):
        L = element.length()
        nd = element.numberOfDOFs
        fefs = np.zeros(nd)
        dofs_to_idx = element.dofs_to_vector_index

        if (NODE_i, ux) and self.wx != 0.0:    # checks just to be sure
            f_i = self.px * (L-self.a)/L
            f_j = self.px * self.a/L

            fefs[dofs_to_idx[(NODE_i, ux)]] -= f_i
            fefs[dofs_to_idx[(NODE_j, ux)]] -= f_j

        if self.py != 0.0:
            f_i = self.py * (L-self.a)/L
            f_j = self.py * self.a/L

            fefs[dofs_to_idx[(NODE_i, uy)]] -= f_i
            fefs[dofs_to_idx[(NODE_j, uy)]] -= f_j

        if self.pz != 0.0:
            f_i = self.pz * (L-self.a)/L
            f_j = self.pz * self.a/L

            fefs[dofs_to_idx[(NODE_i, uz)]] -= f_i
            fefs[dofs_to_idx[(NODE_j, uz)]] -= f_j

        return fefs
    
    # Local y
    def shear_y(self, x, element):
        L = element.length()
        a = self.a
        Vy = 0.0

        # discontinuous at x = a
        if x < a:
            Vy = self.py * (L-a)/L
        if x > a:
            Vy = self.py * a/L
        return Vy
    
    def moment_z(self, x, element):
        L = element.length()
        a = self.a
        Mz = 0.0

        if x == a:
            Mz = self.py * a * (L-a) / L
        if x < a:
            Mz = self.py * x * (L-a) / L
        if x > a:
            Mz = self.py * a * (L-x) / L
        return Mz

    # Local z    
    def shear_z(self, x, element):
        L = element.length()
        a = self.a
        Vz = 0.0

        # discontinuous at x = a
        if x < a:
            Vz = self.pz * (L-a)/L
        if x > a:
            Vz = self.pz * a/L
        return Vz
    
    def moment_y(self, x, element):
        L = element.length()
        a = self.a
        My = 0.0

        if x == a:
            My = self.pz * a * (L-a) / L
        if x < a:
            My = self.pz * x * (L-a) / L
        if x > a:
            My = self.pz * a * (L-x) / L
        return My
    
    # Local x
    def axial(self, x, element):
        L = element.length()
        a = self.a
        Nx = 0.0

        # discontinuous at x = a
        if x < a:
            Nx = self.px * (L-a)/L
        if x > a:
            Nx = self.px * a/L
        return Nx
    

class PolynomialLoad(ElementLoad):
    # use integration to compute fef
    pass