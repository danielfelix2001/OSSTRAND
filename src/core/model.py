from src.utils.exceptions import ModelDefinitionError

class Model:
    def __init__(self):
        self.node = {}      # {node_id: Node}
        self.element = {}   # {element_id: Element}
        # self.material = {}
        # self.section = {}

        self.ndof = 0  
        self.restrained_dofs = []
        self.free_dofs = []
        self.K_full = None  
        self.F_full = None  
        self.D_full = None 
        self.reactions = None 

        self.preprocessed = False
    
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

    