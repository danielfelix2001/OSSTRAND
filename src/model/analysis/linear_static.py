# src/model/analysis/linear_static.py

import numpy as np 
from src.utils.exceptions import DOFError
from src.model.model import Model

def assemble_loads(model:Model):
    model.F_full = np.zeros(model.ndof)

    for node in model.node.values():  
        for dof_name, global_dof in node.dofs.items():
            if global_dof is None:
                continue
            model.F_full[global_dof] += node.loads.get(dof_name, 0.0)
    
    # Check for DOF consistency
    for node in model.node.values():
        for dof in node.loads:
            if dof not in node.dofs:
                raise DOFError(
                    f"Node {node.id}: load applied to undefined DOF {dof}"
                )
    
    # Load application
    for node in model.node.values():
        for dof, load in node.loads.items():
            if load != 0.0 and node.restraints.get(dof, False):
                print(
                    f"Warning: load applied at restrained DOF "
                    f"(Node {node.id}, DOF {dof})"
                )

def assemble_fixed_end_forces(model:Model):
    for element in model.element.values():
        if element.fef_local is None: # skip if truss
            continue

        element.compute_fef()
        
        T = element.transformation_matrix()
        fef_global = T.T @ element.fef_local

        dofs = element.get_dof_indices()
        for i, global_dof in enumerate(dofs):
            if global_dof is not None:
                # subtract because FEFs are reactions
                model.F_full[global_dof] -= fef_global[i]

def solve_matrix_equation(model:Model):
    free = model.free_dofs
    K_ff = model.K_full[np.ix_(free, free)]
    F_f  = model.F_full[free]
    D_f = np.linalg.solve(K_ff, F_f)

    model.D_full = np.zeros(model.ndof)
    model.D_full[free] = D_f
    model.reactions = model.K_full @ model.D_full - model.F_full

def store_displacements(model:Model):
    for node in model.node.values():
        for local_dof, global_dof in node.dofs.items():
            if global_dof in model.free_dofs:
                node.displacements[local_dof] = model.D_full[global_dof]
            else:
                node.displacements[local_dof] = 0.0

def store_reactions(model:Model):
    for node in model.node.values():
        for local_dof, global_dof in node.dofs.items():
            if global_dof in model.restrained_dofs:
                node.reactions[local_dof] = model.reactions[global_dof]

def compute_end_forces(model:Model):
    for element in model.element.values():
        if element.fef_local is None: # skip if truss
            continue
        
        dofs = element.get_dof_indices()

        # Extract global displacements for element
        d_global = np.zeros(len(dofs))
        for i, dof in enumerate(dofs):
            if dof is not None:     # dof is none for restrained DOFs
                d_global[i] = model.D_full[dof]

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

def solve(model:Model):
    if not model._preprocessed:
        raise RuntimeError(
            "Model.preprocess() was not called before solve()"
        )
    assemble_loads(model)
    assemble_fixed_end_forces(model)
    solve_matrix_equation(model)
    store_displacements(model)
    store_reactions(model)
    compute_end_forces(model)