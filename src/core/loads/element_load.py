from abc import ABC, abstractmethod
import numpy as np
from src.utils import global_variables as gv

class ElementLoad(ABC):
    @abstractmethod
    def fef_local(self):
        """
        Returns the local basis fixed end forces (reaction) of the element load in vector form.   
        """
        pass

    def ref_local(self):
        """
        Returns the local end-force (reaction) vector of the element load,
        accounting for released DOFs.   
        """
        element = self.element

        kept, released = element.kept_and_released_indices()
        if not released:
            return self.fef_local()

        dofs_to_vctr_idx = element.dofs_to_vctr_idx 
        kept_vctr = [dofs_to_vctr_idx[(node, dof)] for node, dof in kept]
        released_vctr = [dofs_to_vctr_idx[(node, dof)] for node, dof in released]

        k_local = element.local_stiffness()
        k_kr = k_local[np.ix_(kept_vctr, list(released_vctr))]
        k_rr = k_local[np.ix_(list(released_vctr), list(released_vctr))]

        fef_local = self.fef_local()
        f_k = fef_local[kept_vctr]
        f_r = fef_local[released_vctr]

        # Condensed end force vector considering releases
        f_cond = f_k - k_kr @ np.linalg.solve(k_rr, f_r)

        # Expand back to full vector
        f_ref = np.zeros_like(fef_local)
        f_ref[kept_vctr] = f_cond
        #f_ref[released_vctr] = 0.0

        return f_ref
    
    def out_of_bounds(self, x):
        if x > self.L:
            raise ValueError("Distance from node i can not be greater than element length.")
        if x < 0.0:
            raise ValueError("Distance from node i can not be less than zero.")

class UDL(ElementLoad):
    """
    Uniformly Distributed Load
    """
    def __init__(self, id:str, element, local=True, wx=0.0, wy=0.0, wz=0.0):
        self.id = id
        self.element = element 

        # Initialize
        self.applied_load_x_local = 0.0
        self.applied_load_y_local = 0.0
        self.applied_load_z_local = 0.0

        self.isLocal = local
        self.L = self.element.length()

        # Containers
        self.applied_load_x_input = wx
        self.applied_load_y_input = wy
        self.applied_load_z_input = wz
    
        if self.isLocal:
            self.applied_load_x_local = self.applied_load_x_input
            self.applied_load_y_local = self.applied_load_y_input
            self.applied_load_z_local = self.applied_load_z_input

        else:
            x_global = np.array([1.0, 0.0, 0.0])
            y_global = np.array([0.0, 1.0, 0.0])
            z_global = np.array([0.0, 0.0, 1.0])
        
            x_local, y_local, z_local = self.element.local_axes()

            # Decompose Global UDLs to local axes
            # Local x
            self.applied_load_x_local = (self.applied_load_x_input * np.dot(x_global, x_local) +
                       self.applied_load_y_input * np.dot(y_global, x_local) +
                       self.applied_load_z_input * np.dot(z_global, x_local))
            # Local y
            self.applied_load_y_local = (self.applied_load_x_input * np.dot(x_global, y_local) +
                       self.applied_load_y_input * np.dot(y_global, y_local) +
                       self.applied_load_z_input * np.dot(z_global, y_local))
            # Local z
            self.applied_load_z_local = (self.applied_load_x_input * np.dot(x_global, z_local) +
                       self.applied_load_y_input * np.dot(y_global, z_local) +
                       self.applied_load_z_input * np.dot(z_global, z_local))  
    
    def load_factored(self, load_factor:float)-> "UDL":
        return UDL(
            id = self.id,
            element = self.element,
            wx = self.applied_load_x_local * load_factor,
            wy = self.applied_load_y_local * load_factor,
            wz = self.applied_load_z_local * load_factor
        )

    def required_local_dofs(self):
            req = set()

            if self.applied_load_x_local != 0.0:
                req |= {(gv.NODE_i, gv.ux), (gv.NODE_j, gv.ux)}

            if self.applied_load_y_local != 0.0:
                req |= {
                    (gv.NODE_i, gv.uy), (gv.NODE_j, gv.uy),
                    (gv.NODE_i, gv.rz), (gv.NODE_j, gv.rz),
                }

            if self.applied_load_z_local != 0.0:
                req |= {
                    (gv.NODE_i, gv.uz), (gv.NODE_j, gv.uz),
                    (gv.NODE_i, gv.ry), (gv.NODE_j, gv.ry),
                }

            return req

    def fef_local(self):
        L = self.L
        nd = self.element.numberOfDOFs
        fefs = np.zeros(nd)
        dofs_to_vctr_idx = self.element.dofs_to_vctr_idx       

        # ---- Local x load ----
        if self.applied_load_x_local != 0.0:    # checks just to be sure
            f = self.applied_load_x_local * L / 2

            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.ux)]] -= f
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.ux)]] -= f

        # ---- Local y load → bending about z ----
        if self.applied_load_y_local != 0.0:
            f = self.applied_load_y_local * L / 2
            m = self.applied_load_y_local * L**2 / 12
            
            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.uy)]]  -= f     # dof[(node, dof_type)]
            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.rz)]]  -= m     # node i = 0
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.uy)]]  -= f     # node j = 1
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.rz)]]  += m

        # ---- Local z load → bending about y ----
        if self.applied_load_z_local != 0.0:
            f = self.applied_load_z_local * L / 2
            m = self.applied_load_z_local * L**2 / 12

            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.uz)]] -= f
            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.ry)]] += m
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.uz)]] -= f
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.ry)]] -= m

        return fefs
    
    # Internal Force Contribution
    # Local x
    def axial(self, x):
        self.out_of_bounds(x)
        return self.applied_load_x_local * x
    def torsion(self, x):
        return 0.0

    # Local y 
    def shear_y(self, x):
        self.out_of_bounds(x)
        return self.applied_load_y_local * x
    def moment_z(self, x):
        self.out_of_bounds(x)
        return 0.5 * self.applied_load_y_local * x**2

    # Local z
    def shear_z(self, x):
        self.out_of_bounds(x)
        return self.applied_load_z_local * x
    def moment_y(self, x):
        self.out_of_bounds(x)
        return 0.5 * self.applied_load_z_local * x**2
    
class SelfWeight(ElementLoad):  
    def __init__(self, id:str, element, scale=1.0):
        self.id = id
        self.element = element
        self.L = self.element.length()

        gravity_vector = np.array([0.0, -1.0, 0.0])
        x_local, y_local, z_local = element.local_axes()
        selfWeight = element.material.gamma * element.section.area * scale

        # Decompose self weight to local x,y,z
        self.applied_load_x_local = selfWeight * np.dot(gravity_vector, x_local)
        self.applied_load_y_local = selfWeight * np.dot(gravity_vector, y_local)
        self.applied_load_z_local = selfWeight * np.dot(gravity_vector, z_local)

    def load_factored(self, load_factor:float)-> "SelfWeight":
        return SelfWeight(
            id = self.id, 
            element = self.element, 
            scale = load_factor
        )

    def required_local_dofs(self):
            req = set()

            if self.applied_load_x_local != 0.0:
                req |= {(gv.NODE_i, gv.ux), (gv.NODE_j, gv.ux)}

            if self.applied_load_y_local != 0.0:
                req |= {
                    (gv.NODE_i, gv.uy), (gv.NODE_j, gv.uy),
                    (gv.NODE_i, gv.rz), (gv.NODE_j, gv.rz),
                }

            if self.applied_load_z_local != 0.0:
                req |= {
                    (gv.NODE_i, gv.uz), (gv.NODE_j, gv.uz),
                    (gv.NODE_i, gv.ry), (gv.NODE_j, gv.ry),
                }

            return req
    
    def fef_local(self):
        L = self.L
        nd = self.element.numberOfDOFs
        fefs = np.zeros(nd)
        dofs_to_vctr_idx = self.element.dofs_to_vctr_idx
        
        # ---- Local y load bending about z ----
        if self.applied_load_y_local != 0.0:
            f = self.applied_load_y_local * L / 2
            m = self.applied_load_y_local * L**2 / 12

            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.uy)]]  -= f     # dof[(node, dof_type)]
            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.rz)]]  -= m     # node i = 0
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.uy)]]  -= f     # node j = 1
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.rz)]]  += m

        # ---- Local z load bending about y ----
        if self.applied_load_z_local != 0.0:
            f = self.applied_load_z_local * L / 2
            m = self.applied_load_z_local * L**2 / 12

            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.uz)]] -= f
            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.ry)]] += m
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.uz)]] -= f
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.ry)]] -= m

        # ---- Local x load ----
        if self.applied_load_x_local != 0.0:    
            f = self.applied_load_x_local * L / 2

            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.ux)]] -= f
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.ux)]] -= f

        return fefs

    # Internal Force Contribution
    # Local y 
    def shear_y(self, x):
        self.out_of_bounds(x)
        return self.applied_load_y_local * x
    def moment_z(self, x):
        self.out_of_bounds(x)
        return 0.5 * self.applied_load_y_local * x**2

    # Local z
    def shear_z(self, x):
        self.out_of_bounds(x)
        return self.applied_load_z_local * x
    def moment_y(self, x):
        self.out_of_bounds(x)
        return 0.5 * self.applied_load_z_local * x**2
    
    # Local x
    def axial(self, x):
        self.out_of_bounds(x)
        return self.applied_load_x_local * x
    def torsion(self, x):
        return 0.0

class PointLoad(ElementLoad):
    def __init__(self, id:str, element, dist_from_NODE_i:float, local=True, px=0.0, py=0.0, pz=0.0):
        self.id = id
        self.element = element
        self.applied_load_x_local = 0.0
        self.applied_load_y_local = 0.0
        self.applied_load_z_local = 0.0
        self.a  = dist_from_NODE_i
        self.isLocal = local
        self.L = self.element.length()

        # Containers
        self.applied_load_x_input = px
        self.applied_load_y_input = py
        self.applied_load_z_input = pz

        # Validity
        if self.a > self.L:
            raise ValueError(
                "Distance from node i can not be greater than element length."
            )
        elif self.a == self.L:
            raise ValueError(
                "Distance from node i can not be equal to element length.\n Use NodalLoad instead."
            )

        if self.isLocal:
            self.applied_load_x_local = self.applied_load_x_input
            self.applied_load_y_local = self.applied_load_y_input
            self.applied_load_z_local = self.applied_load_z_input
            
        else:
            x_global = np.array([1.0, 0.0, 0.0])
            y_global = np.array([0.0, 1.0, 0.0])
            z_global = np.array([0.0, 0.0, 1.0])
        
            x_local, y_local, z_local = self.element.local_axes()

            # Decompose Global point loads to local axes
            # Local x
            self.applied_load_x_local = (self.applied_load_x_input * np.dot(x_global, x_local) +
                        self.applied_load_y_input * np.dot(y_global, x_local) +
                        self.applied_load_z_input * np.dot(z_global, x_local))
            # Local y
            self.applied_load_y_local = (self.applied_load_x_input * np.dot(x_global, y_local) +
                       self.applied_load_y_input * np.dot(y_global, y_local) +
                       self.applied_load_z_input * np.dot(z_global, y_local))
            # Local z
            self.applied_load_z_local = (self.applied_load_x_input * np.dot(x_global, z_local) +
                       self.applied_load_y_input * np.dot(y_global, z_local) +
                       self.applied_load_z_input * np.dot(z_global, z_local)) 
            
    def load_factored(self, load_factor:float)-> "PointLoad":
        return PointLoad(
            id = self.id,
            element = self.element,
            px = self.applied_load_x_local * load_factor,
            py = self.applied_load_y_local * load_factor,
            pz = self.applied_load_z_local * load_factor 
        )
    
    def required_local_dofs(self):
            req = set()

            if self.applied_load_x_local != 0.0:
                req |= {(gv.NODE_i, gv.ux), (gv.NODE_j, gv.ux)}

            if self.applied_load_y_local != 0.0:
                req |= {
                    (gv.NODE_i, gv.uy), (gv.NODE_j, gv.uy),
                    (gv.NODE_i, gv.rz), (gv.NODE_j, gv.rz),
                }

            if self.applied_load_z_local != 0.0:
                req |= {
                    (gv.NODE_i, gv.uz), (gv.NODE_j, gv.uz),
                    (gv.NODE_i, gv.ry), (gv.NODE_j, gv.ry),
                }

            return req

    def fef_local(self):
        L = self.L
        a = self.a       
        b = L-a
        nd = self.element.numberOfDOFs
        fefs = np.zeros(nd)
        dofs_to_vctr_idx = self.element.dofs_to_vctr_idx            

        if (gv.NODE_i, gv.ux) in dofs_to_vctr_idx and self.applied_load_x_local != 0.0:    # checks just to be sure
            f_i = self.applied_load_x_local * (L-self.a)/L
            f_j = self.applied_load_x_local * self.a/L

            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.ux)]] -= f_i
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.ux)]] -= f_j

        if self.applied_load_y_local != 0.0:
            f_i = self.applied_load_y_local * b**2 * (3*a + b) / L**3
            m_i = self.applied_load_y_local * a * b**2 / L**2
            f_j = self.applied_load_y_local * a**2 * (3*b + a) / L**3
            m_j = self.applied_load_y_local * b * a**2 / L**2

            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.uy)]] -= f_i
            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.rz)]] -= m_i 
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.uy)]] -= f_j
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.rz)]] += m_j

        if self.applied_load_z_local != 0.0:
            f_i = self.applied_load_z_local * b**2 * (3*a + b) / L**3
            m_i = self.applied_load_z_local * a * b**2 / L**2
            f_j = self.applied_load_z_local * a**2 * (3*b + a) / L**3
            m_j = self.applied_load_z_local * b * a**2 / L**2

            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.uy)]] -= f_i
            fefs[dofs_to_vctr_idx[(gv.NODE_i, gv.rz)]] += m_i 
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.uy)]] -= f_j
            fefs[dofs_to_vctr_idx[(gv.NODE_j, gv.rz)]] -= m_j

        return fefs

    # Internal Force Contribution
    # Local y
    def shear_y(self, x):
        self.out_of_bounds(x)
        Vy = 0.0
        # discontinuous at x = a
        if x < self.a:
            Vy = 0.0
        if x > self.a:
            Vy = self.applied_load_y_local
        return Vy
    
    def moment_z(self, x):
        self.out_of_bounds(x)
        Mz = 0.0
        # discontinuous at x = a
        if x < self.a:
            Mz = 0
        if x > self.a:
            Mz = self.applied_load_y_local * (x-self.a)
        return Mz

    # Local z    
    def shear_z(self, x):
        self.out_of_bounds(x)
        Vz = 0.0
        # discontinuous at x = a
        if x < self.a:
            Vz = 0.0
        if x > self.a:
            Vz = self.applied_load_z_local
        return Vz
    
    def moment_y(self, x):
        self.out_of_bounds(x)
        My = 0.0
        # discontinuous at x = a
        if x < self.a:
            My = 0
        if x > self.a:
            My = self.applied_load_z_local * (x-self.a)
        return My
    
    # Local x
    def axial(self, x):
        self.out_of_bounds(x)
        Nx = 0.0
        # discontinuous at x = a
        if x < self.a:
            Nx = 0.0
        if x > self.a:
            Nx = self.applied_load_x_local
        return Nx 
    
    def torsion(self, x):
        return 0.0