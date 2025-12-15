#src/model/materials.py

class Material:
    def __init__(self, material_id: str, 
                 density_KG_PER_M3: float, 
                 E: float, 
                 nu: float):
        self.id = material_id
        self.density_KG_PER_M3 = density_KG_PER_M3  # in kg/m^3
        self.E = E  # in MPa
        self.nu = nu  # Poisson's ratio

    @property #decorator to make G a "variable" instead of a method
    def G(self):
        return self.E / (2 * (1 + self.nu))
    
    @property
    def unit_weight_N_PER_MM3(self):
        return self.density_KG_PER_M3 * 9.81e-9  # in N/mm^3
    
    def __repr__(self):
        return (f"Material(id={self.id}, E={self.E}, "
                f"nu={self.nu}, density={self.density_KG_PER_M3})")
    
class SteelMaterial(Material):
    def __init__(self, material_id: str,
                 E: float,
                 nu: float,
                 density_KG_PER_M3: float,
                 fy: float,
                 fu: float | None = None):
        super().__init__(material_id, density_KG_PER_M3, E, nu)
        self.fy = fy
        self.fu = fu

    def __repr__(self):
        return (f"SteelMaterial(id={self.id}, E={self.E}, "
                f"fy={self.fy}, density={self.density_KG_PER_M3})")
