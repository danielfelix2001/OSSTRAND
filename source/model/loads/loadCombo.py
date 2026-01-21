from source.model.fixedEndForces.fefs import UniformlyDistributedLoad, SelfWeight, PointLoad

class NodalLoad:
    def __init__(self, node, dof, magnitude:float):
        self.node = node
        self.dof = dof
        self.magnitude = magnitude
    
    def apply(self, loadFactor):
        self.node.add_load(self.dof, self.magnitude*loadFactor)
    
class ElementLoad:
    pass

class UDL(ElementLoad):
    def __init__(self, element, local, wx=0.0, wy=0.0, wz=0.0):
        self.element = element 
        self.wx = wx
        self.wy = wy
        self.wz = wz
        self.isLocal = local
    
    def apply(self, loadFactor):
        self.element.add_load(
            UniformlyDistributedLoad(
                local = self.isLocal,
                wx = self.wx * loadFactor,
                wy = self.wy * loadFactor,
                wz = self.wz * loadFactor
            )
        )

class SlfWgt(ElementLoad):
    def __init__(self, element):
        self.element = element
    
    def apply(self, loadFactor):
        self.element.add_load(
            SelfWeight(loadFactor)
        )

class PntLd(ElementLoad):
    def __init__(self, element, NODE_i_DISTANCE, local=True, px=0.0, py=0.0, pz=0.0):
        self.element = element
        self.px = px
        self.py = py
        self.pz = pz
        self.a  = NODE_i_DISTANCE
        self.isLocal = local

    def apply(self, loadFactor):
        self.element.add_load(
            PointLoad(
                NODE_i_DISTANCE = self.a,
                local = self.isLocal,
                px = self.px*loadFactor,
                py = self.py*loadFactor,
                pz = self.pz*loadFactor
            )
        )

class LoadCombination:
    def __init__(self, name:str, loadCaseAndFactors):
        """
        factors: dict {load_case_name: factor}
        example: {DL:1.2, LL:1.6}
        """
        self.name = name
        self.loadCaseAndFactors = loadCaseAndFactors

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