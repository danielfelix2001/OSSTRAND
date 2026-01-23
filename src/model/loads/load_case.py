# src/model/loads/load_case.py

from src.model.loads.nodal_load import NodalLoad
from src.model.loads.element_load import ElementLoad

class LoadCase:
    def __init__(self, name: str):
        self.name = name
        self.nodalLoads = []     
        self.elementLoads = []   

    def add_nodal_load(self, load:NodalLoad):
        if not isinstance(load, NodalLoad):
            raise TypeError(f"{load} is not a Nodal Load")
        
        self.nodalLoads.append(load)

    def add_element_load(self, load:ElementLoad):
        if not isinstance(load, ElementLoad):
            raise TypeError(f"{load} is not an Element Load")
        
        self.elementLoads.append(load)