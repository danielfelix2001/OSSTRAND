# src/model/loads/element_load.py

from abc import ABC, abstractmethod
from src.model.loads.fixed_end_forces import UniformlyDistributedLoad, SelfWeight, PointLoad

class ElementLoad(ABC):
    @abstractmethod
    def apply(self, loadFactor):
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