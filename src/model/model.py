# src/model/model.py

from src.utils.exceptions import ModelDefinitionError

class Model:
    def __init__(self):
        self.node = {}
        self.element = {}
        self.material = {}
        self.section = {}

        self.ndof = 0  
        self.restrained_dofs = []
        self.free_dofs = []
        self.K_full = None  
        self.F_full = None  
        self.D_full = None 
        self.reactions = None 

        self._preprocessed = False
    
    # Objects
    def add_node(self, node):
        if node.id in self.node:
            raise ModelDefinitionError(
                f"Duplicate node ID detected: {node.id}"
            )
        self.node[node.id] = node
    
    def add_element(self, element):
        if element.id in self.node:
            raise ModelDefinitionError(
                f"Duplicate element ID detected: {element.id}"
            )
        self.element[element.id] = element

#   def add_material(self, material):
#       self.material[material.id] = material

#   def add_section(self, section):
#       self.section[section.id] = section

    def preprocess(self):
        """
        Validates model topology  
        Assigns DOFs    
        Assemble model stiffness    
        Checks stability
        """
        from src.model.analysis.preprocessing import preprocess as _preprocess
        _preprocess(self)
    
    def apply_loads_in_load_combo(self, load_combo):
        # reset all nodes and elements between each load combo application
        for node in self.node.values():
            node.reset() 
        for element in self.element.values():
            element.reset()

        for loadCase, loadFactor in load_combo.loadCaseAndFactors.items():
            if not load_combo.loadCaseAndFactors:
                raise ValueError("Load combination must have at least one load case and factor.")

            # apply load factors
            for nodalLoad in loadCase.nodalLoads:
                nodalLoad.apply(loadFactor)        
            for elementLoad in loadCase.elementLoads:
                elementLoad.apply(loadFactor)

    def linear_static_solve(self, load_combo):
        from src.model.analysis.linear_static import solve as _solve

        self.apply_loads_in_load_combo(load_combo) 
        _solve(self)