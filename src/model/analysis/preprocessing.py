# src/model/analysis/preprocessing.py

from src.model.model import Model
import numpy as np
from src.utils.exceptions import ModelDefinitionError, StabilityError, ElementError


def validate_model(model:Model):
    # check if nodes, elements exist 
    if not model.node:
        raise ModelDefinitionError("Model has no nodes.")

    if not model.element:
        raise ModelDefinitionError("Model has no elements.")      
    
    # check element connectivity
    for element in model.element.values():
        if element.i not in model.node.values():
            raise ElementError(f"Element {element.id} has invalid start node.")
        if element.j not in model.node.values():
            raise ElementError(f"Element {element.id} has invalid end node.")
        if element.i is element.j:
            raise ElementError(f"Element {element.id} has zero connectivity (i == j).")

    # check element properties
    for element in model.element.values():
        if hasattr(element, "E") and element.E <= 0:
            raise ElementError(f"Element {element.id}: invalid E.")
        if hasattr(element, "A") and element.A <= 0:
            raise ElementError(f"Element {element.id}: invalid A.")

def assign_dofs(model:Model):
    model.free_dofs = []
    model.restrained_dofs = []

    dof_counter = 0
    for node in model.node.values():
        node.dofs = {}

    # DOF declaration 
    # Elements dictate what DOFs are available for the node
    for element in model.element.values():
        for node in (element.i, element.j):
            for dof in element.NODE_DOF_INDICES:
                if dof not in node.dofs:
                    node.dofs[dof] = None   # make local dof key with None value
                    node.restraints.setdefault(dof, False)
    
    # Numbering Phase
    for node in model.node.values():                   
        for dof_name in sorted(node.dofs.keys()):
            # Assign model-level DOF value to key dof_name
            node.dofs[dof_name] = dof_counter   

            # DOF is either restrained or free
            if node.restraints.get(dof_name, False):
                model.restrained_dofs.append(dof_counter)
            else:
                model.free_dofs.append(dof_counter)
            
            dof_counter += 1
    model.ndof = dof_counter

def assemble_stiffness(model:Model):
    model.K_full = np.zeros((model.ndof, model.ndof))

    for element in model.element.values():
        K = element.global_stiffness()
        dofs = element.get_dof_indices() # available DOFs from the element 
        nd = len(dofs)
        
        for i in range(nd):     # i and j are the global stiffness indices (row, col)
            for j in range(nd):
                if dofs[i] is not None and dofs[j] is not None:
                    model.K_full[dofs[i], dofs[j]] += K[i, j]

def check_stability(model:Model):
    tol=1e-8
    # Extract free–free stiffness matrix
    free = model.free_dofs
    if len(free) == 0:
        raise StabilityError("No free DOFs in model.")

    K_ff = model.K_full[np.ix_(free, free)]

    dof_map = {}
    for node in model.node.values():
        for dof_name, gidx in node.dofs.items():
            if gidx in free:
                dof_map[gidx] = (node.id, dof_name)

    # Zero row/column check
    zero_rows = []
    for i in range(K_ff.shape[0]):
        if np.all(np.abs(K_ff[i, :]) < tol):
            zero_rows.append(free[i])

    if zero_rows:
        for dof in zero_rows:
            node_id, dof_name = dof_map[dof]
            msg += f"  Node {node_id}, DOF {dof_name}\n"
        raise StabilityError("Zero stiffness detected at DOFs:\n")
    
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

def preprocess(model:Model):
    validate_model(model)
    assign_dofs(model)
    assemble_stiffness(model)
    check_stability(model)
    model._preprocessed = True