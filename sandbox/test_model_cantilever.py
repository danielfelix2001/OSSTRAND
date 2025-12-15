from src.model.nodes import Node
from src.model.elements import Element
from src.model.materials import Material, SteelMaterial
from src.model.sections import Section, SteelSection
from src.model.model import Model
import numpy as np

"""
Store nodes
Store materials
Store sections
Store elements
Assign global DOF numbers
Assemble K and F
Call the solver
"""

"""
Global xyz system
x to right
y up
z backward
"""
# --------------------------------
# MODEL
# --------------------------------
MODEL_1 = Model()

# --------------------------------
# NODES
# --------------------------------
n1 = Node(1, 0.0, 0.0, 0.0)
n2 = Node(2, 5000.0, 0.0, 0.0)

# Fix node 1 in all directions
n1.restrain(True, True, True, True, True, True)

# Loads
n2.load(0.0, -1000.0, 0.0, 0.0, 0.0, 0.0)

# Add nodes to model
MODEL_1.add_node(n1)
MODEL_1.add_node(n2)

# --------------------------------
# MATERIAL AND SECTION
# should come from a database in the future
# --------------------------------

STEEL_1 = SteelMaterial(
    material_id = "A36",
    density_KG_PER_M3 = 7850,   #kg/m^3
    nu = 0.30,
    E = 200000,        #MPa
    fy = 250,          #MPa
    fu = 400           #MPa
)

SECTION_1 = SteelSection(
    section_id = "W200x15", #W8x10 in english units
    shape_type = "W",
    area = 1910,     #mm^2
    Ixx = 12.8e+06,  #mm^4
    Iyy = 0.87e+06,  #mm^4
    J = 17.7e+03,    #mm^4
    Sx = 128e+03,    #mm^3
    Sy = 17.4e+03,   #mm^3
    Zx = 145e+03,    #mm^3
    Zy = 27.2e+03    #mm^3
)

# --------------------------------
# ELEMENTS
# --------------------------------
ELEMENT_1 = Element(
    element_id = "1,2",
    node_i = n1,
    node_j = n2,
    material = STEEL_1,
    section = SECTION_1,
    roll_radians = 0.0
)

# Add UDL to element
ELEMENT_1.add_udl(qy = -1.0) # N/mm, local y

MODEL_1.add_element(ELEMENT_1)

# --------------------------------
# ASSEMBLY AND SOLVING
# --------------------------------
MODEL_1.assign_dofs()
MODEL_1.assemble_stiffness()
MODEL_1.assemble_loads()
MODEL_1.assemble_fixed_end_forces()
MODEL_1.solve()
MODEL_1.store_displacements()
MODEL_1.store_reactions()

print(n1.reactions)
print(n2.displacements)



