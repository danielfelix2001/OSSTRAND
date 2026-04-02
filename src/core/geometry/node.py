class Node:
    def __init__(self, node_id:str, 
                 x: float, y: float, z: float = 0.0):
        self.id = node_id
        self.x, self.y, self.z = x, y, z

        self.dofs = {}          # {Local-level DOF index : Model-level DOF index} 
        self.restraints = {}    # {Local-level DOF index : True or False} 

    def restrain(self, dof_name):
        self.restraints[dof_name] = True 
