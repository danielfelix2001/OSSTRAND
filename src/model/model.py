#src/model/model.py

from src.model.elements import Element
import numpy as np

class Model:
    def __init__(self):
        self.nodes = {}
        self.elements = {}
        self.materials = {}
        self.sections = {}

        self.ndof = 0  
        self.restrained_dofs = []
        self.free_dofs = []
        self.K_full = None  
        self.F_full = None  
        self.D_full = None 
        self.reactions = None 
    
    # Objects
    def add_node(self, node):
        self.nodes[node.id] = node
    
    def add_element(self, element):
        self.elements[element.id] = element

    def add_material(self, material):
        self.materials[material.id] = material

    def add_section(self, section):
        self.sections[section.id] = section

    # DOF Assignment
    def assign_dofs(self):
        dof_counter = 0  #numbering starts at 0
        for node in self.nodes.values(): 
            for i in range(6):
                node.dof[i] = dof_counter
                if node.restraints[i]:
                    self.restrained_dofs.append(dof_counter)
                else:
                    self.free_dofs.append(dof_counter)
                dof_counter += 1
        self.ndof = dof_counter

    def assemble_stiffness(self):
        self.K_full = np.zeros((self.ndof, self.ndof))

        for element in self.elements.values():
            K = element.global_stiffness()
            dofs = element.get_dof_indices()

            for i in range(12):
                for j in range(12):
                    if dofs[i] is not None and dofs[j] is not None:
                        self.K_full[dofs[i], dofs[j]] += K[i, j]

    def assemble_loads(self):
        self.F_full = np.zeros(self.ndof)

        for node in self.nodes.values():  
            for i, dof in enumerate(node.dof):
                if dof is not None:
                    self.F_full[dof] += node.loads[i]

    def assemble_fixed_end_forces(self):
        for element in self.elements.values():
            fef_local = element.fef_local
            T = element.transformation_matrix()
            fef_global = T.T @ fef_local

            dofs = element.get_dof_indices()

            for i in range(12):
                if dofs[i] is not None:
                    # SUBTRACT fixed-end forces
                    self.F_full[dofs[i]] -= fef_global[i]

    def solve(self):
        free = self.free_dofs

        K_ff = self.K_full[np.ix_(free, free)]
        F_f  = self.F_full[free]

        D_f = np.linalg.solve(K_ff, F_f)

        self.D_full = np.zeros(self.ndof)
        self.D_full[free] = D_f
        self.reactions = self.K_full @ self.D_full - self.F_full

    def store_displacements(self):
        for node in self.nodes.values():
            node.displacements = [0.0]*6
            for i, dof in enumerate(node.dof):
                if dof is not None:
                    node.displacements[i] = self.D_full[dof] 

    def store_reactions(self):
        for node in self.nodes.values():
            node.reactions = [0.0]*6
            for i, dof in enumerate(node.dof):
                if dof is not None:
                    node.reactions[i] = self.reactions[dof]

    def summary(self):
            print("MODEL SUMMARY")
            print(f"Nodes: {len(self.nodes)}")
            print(f"Elements: {len(self.elements)}")
            print(f"Free DOFs: {self.ndof}")
    
            for node in self.nodes.values():
                print(f"Node {node.id} DOFs:", node.dof)