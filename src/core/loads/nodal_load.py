from src.core.geometry.node import Node

class NodalLoad:
    def __init__(self, id:str, node:Node, dof, magnitude:float):
        self.id = id
        self.node = node
        self.dof = dof
        self.magnitude = magnitude