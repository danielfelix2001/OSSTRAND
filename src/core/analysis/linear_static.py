import numpy as np 
from src.utils.exceptions import DOFError
from src.core.model import Model
from src.core.loads.load_combo import LoadCombination
from src.core.results.solution_state import SolutionState
from collections import defaultdict
            
def model_force_vector(model:Model, load_combo:LoadCombination) -> np.ndarray: 
    model.F_full = np.zeros(model.ndof)

    for load_case, load_factor in load_combo.loadCaseAndFactors.items():
        if not load_combo.loadCaseAndFactors:
            raise ValueError("Load combination must have at least one load case and factor.")

        for nodalLoad in load_case.nodalLoads:
            nodeDOFsDict = nodalLoad.node.dofs  # {local_dof : global_dof_index}
            loadDOF = nodalLoad.dof             # dict key
            
            if loadDOF not in nodeDOFsDict:     
                raise DOFError(
                    f"Node {nodalLoad.node.id}: load applied to undefined DOF {loadDOF}"
                )
            if nodalLoad.node.restraints.get(loadDOF, False): 
                print(
                    f"Warning: load applied at restrained DOF "
                    f"(Node {nodalLoad.node.id}, DOF {loadDOF})"
                )

            global_dof = nodeDOFsDict[loadDOF]  # dict value

            if global_dof is not None: 
                model.F_full[global_dof] += nodalLoad.magnitude * load_factor

        # Apply real-end forces on F_full
        for element_load in load_case.elementLoads:
            element = element_load.element
            dofs_to_vctr_idx = element.dofs_to_vctr_idx 

            # check if element provides DOF for load
            for rqrd_dofs in element_load.required_local_dofs():
                if rqrd_dofs not in dofs_to_vctr_idx:
                    node_label, dof = rqrd_dofs
                    raise DOFError(
                        f"Element {element.id}: load requires DOF {dof} "
                        f"on {node_label}, but element does not provide it"
                    )
                
            T = element.transformation_matrix()            
            ref_local = element_load.ref_local()
            ref_global = T.T @ ref_local
            
            global_dofs = element.get_gdof_indices()
            for vctr_idx, gdof in enumerate(global_dofs):
                if gdof is not None:
                    # subtract because FEFs are reactions
                    model.F_full[gdof] -= ref_global[vctr_idx] * load_factor

    return model.F_full

def element_data(model:Model, load_combo:LoadCombination) -> dict:
    element_end_forces = {}                     # {element.id: f_local}
    element_dof_to_vctr_idx = {}                # {element.id: element.dofs_to_vctr_idx}
    element_load_registry = defaultdict(list)   # {element.id: list[ElementLoad*load_factor]}
    ref_local_sum = {}                          # {element.id: ref_vector}

    for load_case, load_factor in load_combo.loadCaseAndFactors.items():
        for element_load in load_case.elementLoads:
            element = element_load.element 

            # add element load to registry
            factored_element_load = element_load.load_factored(load_factor)
            element_load_registry[element.id].append(factored_element_load)

            # sum element load effects
            ref_local = factored_element_load.ref_local()
            default_vctr_size = len(element.LOCAL_FORCES_PER_NODE)*2
            if element.id not in ref_local_sum:
                ref_local_sum[element.id] = np.zeros(default_vctr_size) 

            ref_local_sum[element.id] += ref_local

    for element in model.element.values():       
        global_dofs = element.get_gdof_indices() 

        d_global = np.zeros(len(global_dofs))
        for vctr_idx, gdof in enumerate(global_dofs):
            if gdof is not None:     # dof is none for restrained DOFs
                d_global[vctr_idx] = model.D_full[gdof]

        k_local = element.local_stiffness()
        T = element.transformation_matrix()
        d_local = T @ d_global

        # Local end forces
        default_vctr_size = len(element.LOCAL_FORCES_PER_NODE)*2
        f_local = k_local @ d_local + ref_local_sum.get(element.id, np.zeros(default_vctr_size))
        
        element_end_forces[element.id] = f_local
        element_dof_to_vctr_idx[element.id] = element.dofs_to_vctr_idx

    return element_end_forces, element_dof_to_vctr_idx, element_load_registry

def LinearStaticSolve(model:Model, load_combo:LoadCombination) -> SolutionState:
    if not model.preprocessed:
        raise RuntimeError(
            "Preprocess(Model) was not called before solve."
        )

    free = model.free_dofs

    F_full = model_force_vector(model, load_combo)
    F_f = F_full[free]
    K_ff = model.K_full[np.ix_(free, free)]
    D_f = np.linalg.solve(K_ff, F_f) 

    model.D_full = np.zeros(model.ndof)
    model.D_full[free] = D_f  # Displacements

    R_full = model.K_full @ model.D_full - F_full # Reactions

    eef, map, element_loads = element_data(model, load_combo) # Element Data
  
    result =  SolutionState(
        model = model, 
        load_combo = load_combo,
        displacements = model.D_full,
        reactions = R_full,
        element_end_forces = eef,
        element_dofs_to_vctr_idx = map,
        element_loads = element_loads
    ) 
    return result