#src/model/nodes.py

class Node:
    def __init__(self, node_id: 
                 int, x: float, y: float, z: float = 0.0):
        self.id = node_id
        self.x, self.y, self.z = x, y, z

        self.dofs = {}
        self.restraints = {}
        self.loads = {}

        self.displacements = {}
        self.reactions = {}

    def restrain(self, dof_name):
        self.restraints[dof_name] = True

    def add_load(self, dof_name: int, value: float):
        self.loads[dof_name] = self.loads.get(dof_name, 0.0) + value

    def __repr__(self):
        return f"Node(id={self.id}, x={self.x}, y={self.y}, z={self.z})"