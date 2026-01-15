#source/model/model.py

import numpy as np
from source.model.exceptions import StabilityError, ModelDefinitionError, ElementError, DOFError

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
        if node.id in self.node:
            raise ModelDefinitionError(
                f"Duplicate node ID detected: {node.id}"
            )
        self.node[node.id] = node
    
    def add_element(self, element):
        if element.id in self.node:
            raise ModelDefinitionError(
                f"Duplicate element ID detected: {element.id}"
            )
        self.element[element.id] = element

    # def add_material(self, material):
    #     self.material[material.id] = material

    # def add_section(self, section):
    #     self.section[section.id] = section

    def validate_model(self):
        # check if nodes, elements exist 
        if not self.node:
            raise ModelDefinitionError("Model has no nodes.")

        if not self.element:
            raise ModelDefinitionError("Model has no elements.")      
        
        # check element connectivity
        for element in self.element.values():
            if element.i not in self.node.values():
                raise ElementError(f"Element {element.id} has invalid start node.")
            if element.j not in self.node.values():
                raise ElementError(f"Element {element.id} has invalid end node.")
            if element.i is element.j:
                raise ElementError(f"Element {element.id} has zero connectivity (i == j).")

        # check element properties
        for element in self.element.values():
            if hasattr(element, "E") and element.E <= 0:
                raise ElementError(f"Element {element.id}: invalid E.")
            if hasattr(element, "A") and element.A <= 0:
                raise ElementError(f"Element {element.id}: invalid A.")

    def collect_node_dofs(self):
        for element in self.element.values():
            for node in (element.i, element.j):
                for dof in element.NODE_DOF_INDICES:
                    if dof not in node.dofs:
                        # element dictates what DOFs are available for the node
                        node.dofs[dof] = None   
                        node.restraints.setdefault(dof, False)

    def assign_dofs(self):
        self.free_dofs = []
        self.restrained_dofs = []

        dof_counter = 0
        for node in self.node.values():
            for dof_name in node.dofs:
                # Assign model-level DOF index 
                node.dofs[dof_name] = dof_counter   

                # DOF is either restrained or free
                if node.restraints.get(dof_name, False):
                    self.restrained_dofs.append(dof_counter)
                else:
                    self.free_dofs.append(dof_counter)
                
                dof_counter += 1
        self.ndof = dof_counter
  
    def assemble_stiffness(self):
        self.K_full = np.zeros((self.ndof, self.ndof))

        for element in self.element.values():
            K = element.global_stiffness()
            dofs = element.get_dof_indices() # available DOFs from the element 
            nd = len(dofs)
            
            for i in range(nd):     # i and j are the global stiffness indices (row, col)
                for j in range(nd):
                    if dofs[i] is not None and dofs[j] is not None:
                        self.K_full[dofs[i], dofs[j]] += K[i, j]

    def assemble_loads(self):
        self.F_full = np.zeros(self.ndof)

        for node in self.node.values():  
            for dof_name, global_dof in node.dofs.items():
                if global_dof is None:
                    continue
                self.F_full[global_dof] += node.loads.get(dof_name, 0.0)
        
        # Check for DOF consistency
        for node in self.node.values():
            for dof in node.loads:
                if dof not in node.dofs:
                    raise DOFError(
                        f"Node {node.id}: load applied to undefined DOF {dof}"
                    )
        
        # Load application
        for node in self.node.values():
            for dof, load in node.loads.items():
                if load != 0.0 and node.restraints.get(dof, False):
                    print(
                        f"Warning: load applied at restrained DOF "
                        f"(Node {node.id}, DOF {dof})"
                    )

    def assemble_fixed_end_forces(self):
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

    def check_stability(self, tol=1e-8):
        # Extract free–free stiffness matrix
        free = self.free_dofs
        if len(free) == 0:
            raise StabilityError("No free DOFs in model.")

        K_ff = self.K_full[np.ix_(free, free)]

        # Zero row/column check
        zero_rows = []
        for i in range(K_ff.shape[0]):
            if np.all(np.abs(K_ff[i, :]) < tol):
                zero_rows.append(free[i])

        if zero_rows:
            for dof in zero_rows:
                node_id, dof_name = self.global_dof_map[dof]
                msg += f"  Node {node_id}, DOF {dof_name}\n"
            raise StabilityError("Zero stiffness detected at DOFs:\n")
        
        dof_map = {}
        for node in self.node.values():
            for dof_name, gidx in node.dofs.items():
                if gidx in free:
                    dof_map[gidx] = (node.id, dof_name)

        #eigvals, eigvecs = np.linalg.eigh(K_ff)
        eigvals = np.linalg.eigvalsh(K_ff)
        unstable_modes = np.where(eigvals < tol)[0]

        if len(unstable_modes) > 0:
            msg = "Unstable structural modes detected.\n"

            # for mode in unstable_modes:
            #     msg += f"Mode {mode + 1}:\n"
            #     vec = eigvecs[:, mode]

            #     for local_i, amp in enumerate(vec):
            #         if abs(amp) > 1e-3:
            #             global_dof = free[local_i]
            #             node_id, dof_name = dof_map[global_dof]
            #             msg += f"  Node {node_id} -> {DOF_NAMES[dof_name]} (amplitude {amp:.3f})\n"
            
            raise StabilityError(msg)
    
    def solve_matrix_equation(self):
        free = self.free_dofs
        K_ff = self.K_full[np.ix_(free, free)]
        F_f  = self.F_full[free]
        D_f = np.linalg.solve(K_ff, F_f)

        self.D_full = np.zeros(self.ndof)
        self.D_full[free] = D_f
        self.reactions = self.K_full @ self.D_full - self.F_full

    def store_displacements(self):
        for node in self.node.values():
            for local_dof, global_dof in node.dofs.items():
                if global_dof in self.free_dofs:
                    node.displacements[local_dof] = self.D_full[global_dof]
                else:
                    node.displacements[local_dof] = 0.0

    def store_reactions(self):
        for node in self.node.values():
            for local_dof, global_dof in node.dofs.items():
                if global_dof in self.restrained_dofs:
                    node.reactions[local_dof] = self.reactions[global_dof]

    def compute_end_forces(self):
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
            if element.fef_local is None: # if truss
                f_local = element.local_stiffness() @ d_local
            else:
                f_local = element.local_stiffness() @ d_local + element.fef_local

            element.end_forces_local[:] = f_local
            element.end_forces_global[:] = T.T @ f_local


    def solve(self):
        self.validate_model()
        self.collect_node_dofs()
        self.assign_dofs()

        self.assemble_stiffness()
        self.assemble_loads()
        self.assemble_fixed_end_forces()

        self.check_stability()

        self.solve_matrix_equation()
        self.store_displacements()
        self.store_reactions()
        self.compute_end_forces()




