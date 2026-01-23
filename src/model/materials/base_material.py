# src/model/materials/base_material.py

class Material:
    def __init__(self, material_id: str, 
                 E: float = 0.0, 
                 G: float | None = None,
                 nu: float = 0.0,
                 gamma: float = 0.0):
        self.id = material_id
        self.E = E  
        self._G = G 
        self.nu = nu  # Poisson's ratio
        self.gamma = gamma  # unit weight

    @property
    def G(self):
        if self._G is not None:
            return self._G
        return self.E / (2 * (1 + self.nu))