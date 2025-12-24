#src/model/model.py

import numpy as np

class Model:
    def __init__(self):
        self.node = {}
        self.element = {}
        self.material = {}
        self.section = {}

        self.ndof = 0  
        self.restrained_dofs = []
        self.free_dofs = []
        self.K_full = None  
        self.F_full = None  
        self.D_full = None 
        self.reactions = None 
    
    # Objects
    def add_node(self, node):
        self.node[node.id] = node
    
    def add_element(self, element):
        self.element[element.id] = element

    def add_material(self, material):
        self.material[material.id] = material

    def add_section(self, section):
        self.section[section.id] = section

    def solve(self):
    #def collect_node_dofs(self):
        for e in self.element.values():
            for node in (e.i, e.j):
                for dof in e.NODE_DOF_INDICES:
                    if dof not in node.dofs:
                        node.dofs[dof] = None
                        node.restraints.setdefault(dof, False)

    #def assign_dofs(self):
        self.free_dofs = []
        self.restrained_dofs = []

        dof_counter = 0
        for node in self.node.values():
            for dof_name in node.dofs:
                node.dofs[dof_name] = dof_counter
                if node.restraints.get(dof_name, False):
                    self.restrained_dofs.append(dof_counter)
                else:
                    self.free_dofs.append(dof_counter)
                dof_counter += 1
        self.ndof = dof_counter
    
    #def assemble_stiffness(self):
        self.K_full = np.zeros((self.ndof, self.ndof))

        for element in self.element.values():
            K = element.global_stiffness()
            dofs = element.get_dof_indices()
            nd = len(dofs)
            
            for i in range(nd):
                for j in range(nd):
                    if dofs[i] is not None and dofs[j] is not None:
                        self.K_full[dofs[i], dofs[j]] += K[i, j]

    #def assemble_loads(self):
        self.F_full = np.zeros(self.ndof)

        for node in self.node.values():  
            for dof_name, global_dof in node.dofs.items():
                if global_dof is None:
                    continue
                self.F_full[global_dof] += node.loads.get(dof_name, 0.0)

    #def assemble_fixed_end_forces(self):
        for element in self.element.values():
            if element.fef_local is None: # skip if truss
                continue

            element.compute_fef()
            
            T = element.transformation_matrix()
            fef_global = T.T @ element.fef_local

            dofs = element.get_dof_indices()
            for i, global_dof in enumerate(dofs):
                if global_dof is not None:
                    # subtract because FEFs are reactions
                    self.F_full[global_dof] -= fef_global[i]

    #def solve(self):
        free = self.free_dofs
        K_ff = self.K_full[np.ix_(free, free)]
        F_f  = self.F_full[free]
        D_f = np.linalg.solve(K_ff, F_f)

        self.D_full = np.zeros(self.ndof)
        self.D_full[free] = D_f
        self.reactions = self.K_full @ self.D_full - self.F_full

    #def store_displacements(self):
        for node in self.node.values():
            for local_dof, global_dof in node.dofs.items():
                if global_dof in self.free_dofs:
                    node.displacements[local_dof] = self.D_full[global_dof]
                else:
                    node.displacements[local_dof] = 0.0

    #def store_reactions(self):
        for node in self.node.values():
            for local_dof, global_dof in node.dofs.items():
                if global_dof in self.restrained_dofs:
                    node.reactions[local_dof] = self.reactions[global_dof]

    #def compute_end_forces(self):
        for element in self.element.values():
            if element.fef_local is None: # skip if truss
                continue
            
            dofs = element.get_dof_indices()

            # Extract global displacements for element
            d_global = np.zeros(len(dofs))
            for i, dof in enumerate(dofs):
                if dof is not None:     # dof is none for restrained DOFs
                    d_global[i] = self.D_full[dof]

            # Transformation to local displacements
            T = element.transformation_matrix()
            d_local = T @ d_global

            # Local end forces
            f_local = element.local_stiffness() @ d_local + element.fef_local
            element.end_forces_local[:] = f_local
            element.end_forces_global[:] = T.T @ f_local
