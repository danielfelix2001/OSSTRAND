#src/model/nodes.py

class Node:
    def __init__(self, node_id: int, x: float, y: float, z: float = 0.0):
        self.id = node_id
        self.x = x
        self.y = y
        self.z = z
        self.restraints = [False, False, False, False, False, False] 
        self.loads = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.displacements = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.reactions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.dof = [None, None, None, None, None, None]

    def restrain(self, ux: bool, uy: bool, uz: bool, rx: bool, ry: bool, rz: bool):
        self.restraints = [ux, uy, uz, rx, ry, rz]

    def load(self, fx: float, fy: float, fz: float, mx: float, my: float, mz: float):
        self.loads = [fx, fy, fz, mx, my, mz]
    
    def __repr__(self):
        return f"Node(id={self.id}, x={self.x}, y={self.y}, z={self.z})"