#src/model/fixedEndForces/fefs.py

from abc import ABC, abstractmethod
import numpy as np

UX = 0
UY = 1
UZ = 2
RX = 3
RY = 4
RZ = 5

class ElementLoad(ABC):
    @abstractmethod
    def fef_local(self, element):
        """
        Returns local fixed-end force vector in local coordinates
        Size depends on element type
        """
        pass
    
class UDL(ElementLoad):
    def __init__(self, qy=0.0, qz=0.0):
        self.qy = qy
        self.qz = qz

    def fef_local(self, element):
        L = element.length()
        nd = element.ndof_element
        fef = np.zeros(nd)
        dof = element.local_dof_map()

        # ---- Local y load → bending about z ----
        if self.qy != 0.0:
            f = self.qy * L / 2
            m = self.qy * L**2 / 12

            fef[dof[(0, UY)]]  -= f     #dof[(node, dof_type)]
            fef[dof[(0, RZ)]]  -= m
            fef[dof[(1, UY)]]  -= f
            fef[dof[(1, RZ)]] += m

        # ---- Local z load → bending about y ----
        if self.qz != 0.0:
            f = self.qz * L / 2
            m = self.qz * L**2 / 12

            fef[dof[(0, UZ)]] -= f
            fef[dof[(0, RY)]] += m
            fef[dof[(1, UZ)]] -= f
            fef[dof[(1, RY)]] -= m

        return fef
