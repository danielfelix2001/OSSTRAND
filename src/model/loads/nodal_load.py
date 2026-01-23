# src/model/loads/nodal_load.py

from src.model.geometry.node import Node

class NodalLoad:
    def __init__(self, node:Node, dof, magnitude:float):
        self.node = node
        self.dof = dof
        self.magnitude = magnitude
    
    def apply(self, loadFactor):
        self.node.add_load(self.dof, self.magnitude*loadFactor)