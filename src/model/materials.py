#src/model/materials.py

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
        
    def __repr__(self):
        return (f"Material(id={self.id}, E={self.E}, "
                f"nu={self.nu}, gamma={self.gamma})")
    
class SteelMaterial(Material):
    def __init__(self, material_id: str,
                 E: float = 0.0,
                 G: float | None = None,
                 nu: float = 0.0,
                 gamma: float = 0.0,
                 fy: float | None = None,
                 fu: float | None = None):
        super().__init__(material_id, E, G, nu, gamma)
        self.fy = fy
        self.fu = fu

    def __repr__(self):
        return (f"SteelMaterial(id={self.id}, E={self.E}, "
                f"fy={self.fy}, gamma={self.gamma})")
