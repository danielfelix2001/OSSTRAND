# src/model/geometry/node.py

class Node:
    def __init__(self, node_id: 
                 int, x: float, y: float, z: float = 0.0):
        self.id = node_id
        self.x, self.y, self.z = x, y, z

        self.dofs = {}  # Model-level DOF index 
        self.restraints = {}
        self.loads = {}

        self.displacements = {}
        self.reactions = {}
        

    def restrain(self, dof_name):
        self.restraints[dof_name] = True

    def add_load(self, dof_name: int, value: float):
        self.loads[dof_name] = self.loads.get(dof_name, 0.0) + value
    
    def reset(self):
        self.loads = {}
        self.displacements = {}
        self.reactions = {}

    # --------------------------------
    # QUERYING API
    # --------------------------------
    def DISPLACEMENT(self, dof):
        return self.displacements[dof]

    def REACTION(self, dof):
        return self.reactions[dof]