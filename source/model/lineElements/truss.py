#source/model/lineElements/truss.py

from source.model.lineElements.element import Element
import numpy as np

class Truss(Element):
    DOFS_PER_NODE = ["ux", "uy", "uz"]
    NODE_DOF_INDICES = [0, 1, 2]

    def __init__(self, element_id, node_i, node_j, material, section):    
        super().__init__(element_id, node_i, node_j, material, section)
        self.fef_local = None
        self.end_forces_local  = np.zeros(6)
        self.end_forces_global = np.zeros(6)

    def transformation_matrix(self):
        x, _, _ = self.local_axes()
        l, m, n = x

        return np.array([
            [ l, m, n, 0, 0, 0],
            [ 0, 0, 0, l, m, n]
        ])

    def local_stiffness(self):
        E = self.material.E
        A = self.section.area
        L = self.length()
        k = E * A / L

        return np.array([[ k, -k],
                         [-k,  k]])
     
     