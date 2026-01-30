from src.core.model import Model
from src.core.loads.load_combo import LoadCombination
from src.core.loads.element_load import ElementLoad
from src.utils import global_variables as gv
import numpy as np

class SolutionState:
    def __init__(self, model:Model, load_combo: LoadCombination,
                 displacements: np.ndarray, reactions: np.ndarray, 
                 element_end_forces: dict, element_dofs_to_vctr_idx: dict,
                 element_loads: dict[str, list[ElementLoad]]):
        
        self.model = model 
        self.load_combo = load_combo
        
        self.displacements = displacements
        self.reactions = reactions
        self.element_end_forces = element_end_forces
        self.element_dofs_to_vctr_idx = element_dofs_to_vctr_idx
        self.element_loads = element_loads

    # Nodes
    def node_displacement(self, node_id:str, dof:int):
        node = self.model.node[node_id]
        if dof not in node.dofs:
            return 0.0
        global_dof = node.dofs[dof]

        if global_dof is None:
            return 0.0
        
        return self.displacements[global_dof]

    def node_reaction(self, node_id:str, dof:int):
        node = self.model.node[node_id]
        if dof not in node.dofs:
            return 0.0
        global_dof = node.dofs[dof]

        if global_dof is None:
            return 0.0
       
        return self.reactions[global_dof]

    # Elements
    def local_element_end_force(self, element_id:str, node:int, dof:int):
        f_local = self.element_end_forces[element_id]
        dofs_to_vctr_idx = self.element_dofs_to_vctr_idx[element_id]
        vctr_idx = dofs_to_vctr_idx[(node, dof)]
        return f_local[vctr_idx]
    

    # Internal Forces (Signed)
    def internal_force_axial(self, element_id:str, dist_from_NODE_i:float):
        Nx_NODE_i = self.local_element_end_force(element_id, gv.NODE_i, gv.ux)
        Nx_LOAD = 0.0
        factored_load_list = self.element_loads[element_id]
        for load in factored_load_list:
            Nx_LOAD += load.axial(dist_from_NODE_i)
        return -(Nx_NODE_i + Nx_LOAD)
    
    def internal_force_torsion(self, element_id:str, dist_from_NODE_i:float):
        Tx_NODE_i = self.local_element_end_force(element_id, gv.NODE_i, gv.rx)
        Tx_LOAD = 0.0
        factored_load_list = self.element_loads[element_id]
        for load in factored_load_list:
            Tx_LOAD += load.torsion(dist_from_NODE_i)
        return -(Tx_NODE_i + Tx_LOAD)

    def internal_force_shear_y(self, element_id:str, dist_from_NODE_i:float):
        Vy_NODE_i = self.local_element_end_force(element_id, gv.NODE_i, gv.uy)
        Vy_LOAD = 0.0
        factored_load_list = self.element_loads[element_id]
        for load in factored_load_list:
            Vy_LOAD += load.shear_y(dist_from_NODE_i)
        return Vy_NODE_i + Vy_LOAD
    
    def internal_force_shear_z(self, element_id:str, dist_from_NODE_i:float):
        Vz_NODE_i = self.local_element_end_force(element_id, gv.NODE_i, gv.uz)
        Vz_LOAD = 0.0
        factored_load_list = self.element_loads[element_id]
        for load in factored_load_list:
            Vz_LOAD += load.shear_z(dist_from_NODE_i)
        return Vz_NODE_i + Vz_LOAD

    def internal_force_moment_y(self, element_id:str, dist_from_NODE_i:float):
        My_NODE_i = self.local_element_end_force(element_id, gv.NODE_i, gv.ry)
        My_LOAD = 0.0
        factored_load_list = self.element_loads[element_id]
        for load in factored_load_list:
            My_LOAD += load.moment_y(dist_from_NODE_i)
        return My_NODE_i + My_LOAD

    def internal_force_moment_z(self, element_id:str, dist_from_NODE_i:float):
        Mz_NODE_i = self.local_element_end_force(element_id, gv.NODE_i, gv.rz)
        Mz_LOAD = 0.0
        factored_load_list = self.element_loads[element_id]
        for load in factored_load_list:
            My_LOAD += load.moment_z(dist_from_NODE_i)
        return Mz_NODE_i + Mz_LOAD
