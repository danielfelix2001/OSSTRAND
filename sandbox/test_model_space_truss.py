from src.model.nodes import Node
from src.model.lineElements.truss import Truss
from src.model.materials import SteelMaterial
from src.model.sections import SteelSection
from src.model.model import Model
from src.model.dofs import DOF_NAMES, REACTION_NAMES

"""
Global xyz system
x to right
y up
z backward
"""

MODEL_1 = Model()

n1 = Node(1,     0.0,     0.0,     0.0)
n2 = Node(2, -2000.0, -8000.0,  4000.0)
n3 = Node(3,  6000.0, -8000.0,  4000.0)
n4 = Node(4,  6000.0, -8000.0, -2000.0)
n5 = Node(5, -2000.0, -8000.0, -2000.0)

n2.restrain(0) #x
n2.restrain(1) #y
n2.restrain(2) #z

n3.restrain(0) #x
n3.restrain(1) #y
n3.restrain(2) #z

n4.restrain(0) #x
n4.restrain(1) #y
n4.restrain(2) #z

n5.restrain(0) #x
n5.restrain(1) #y
n5.restrain(2) #z

# Loads
n1.add_load(0,  200000.0)
n1.add_load(1, -800000.0)
n1.add_load(2, -600000.0)

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
    area = 20000,     #mm^2
    Ixx = 12.8e+06,  #mm^4
    Iyy = 0.87e+06,  #mm^4
    J = 17.7e+03,    #mm^4
    Sx = 128e+03,    #mm^3
    Sy = 17.4e+03,   #mm^3
    Zx = 145e+03,    #mm^3
    Zy = 27.2e+03    #mm^3
)
SECTION_2 = SteelSection(
    section_id = "W200x15", #W8x10 in english units
    shape_type = "W",
    area = 30000,     #mm^2
    Ixx = 12.8e+06,  #mm^4
    Iyy = 0.87e+06,  #mm^4
    J = 17.7e+03,    #mm^4
    Sx = 128e+03,    #mm^3
    Sy = 17.4e+03,   #mm^3
    Zx = 145e+03,    #mm^3
    Zy = 27.2e+03    #mm^3
)
SECTION_3 = SteelSection(
    section_id = "W200x15", #W8x10 in english units
    shape_type = "W",
    area = 40000,     #mm^2
    Ixx = 12.8e+06,  #mm^4
    Iyy = 0.87e+06,  #mm^4
    J = 17.7e+03,    #mm^4
    Sx = 128e+03,    #mm^3
    Sy = 17.4e+03,   #mm^3
    Zx = 145e+03,    #mm^3
    Zy = 27.2e+03    #mm^3
)
SECTION_4 = SteelSection(
    section_id = "W200x15", #W8x10 in english units
    shape_type = "W",
    area = 30000,     #mm^2
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
TRUSS_1 = Truss(
    element_id = "1,2",
    node_i = n1,
    node_j = n2,
    material = STEEL_1,
    section = SECTION_1
)
TRUSS_2 = Truss(
    element_id = "1,3",
    node_i = n1,
    node_j = n3,
    material = STEEL_1,
    section = SECTION_2
)
TRUSS_3 = Truss(
    element_id = "1,4",
    node_i = n1,
    node_j = n4,
    material = STEEL_1,
    section = SECTION_3
)
TRUSS_4 = Truss(
    element_id = "1,5",
    node_i = n1,
    node_j = n5,
    material = STEEL_1,
    section = SECTION_4
)

# --------------------------------
# ASSEMBLY AND SOLVING
# --------------------------------
MODEL_1.add_node(n1)
MODEL_1.add_node(n2)
MODEL_1.add_node(n3)
MODEL_1.add_node(n4)
MODEL_1.add_node(n5)
MODEL_1.add_element(TRUSS_1)
MODEL_1.add_element(TRUSS_2)
MODEL_1.add_element(TRUSS_3)
MODEL_1.add_element(TRUSS_4)

MODEL_1.solve()

# --------------------------------
# RESULTS
# --------------------------------
print("\nNode 1 Displacements:")
for dof, val in n1.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")

print("\nNode 2 Reactions:")
for reactions, val in n2.reactions.items():
    print(f"{REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 3 Reactions:")
for reactions, val in n3.reactions.items():
    print(f"{REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 4 Reactions:")
for reactions, val in n4.reactions.items():
    print(f"{REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 5 Reactions:")
for reactions, val in n5.reactions.items():
    print(f"{REACTION_NAMES[reactions]} = {val:.4e}")




