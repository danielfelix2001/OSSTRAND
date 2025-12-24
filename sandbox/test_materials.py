#sandbox/test_materials.py

import sys
sys.path.append(".")
from source.model.materials import Material, SteelMaterial

Steel1 = SteelMaterial(
    material_id = "A36",
    density_KG_PER_M3 = 7850,   #kg/m^3
    nu = 0.3,
    E = 200e+3,        #MPa
    fy = 250,          #MPa
    fu = 400           #MPa
)
print(Steel1)