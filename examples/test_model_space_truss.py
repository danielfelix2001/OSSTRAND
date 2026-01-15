from source.model.nodes import Node
from source.model.lineElements.truss import Truss
from source.model.materials import SteelMaterial
from source.model.sections import WSection
from source.model.model import Model
from source.model.functions import DOF_NAMES, GLOBAL_REACTION_NAMES, LOCAL_REACTION_NAMES
UX, UY, UZ = 0, 1, 2

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

for n in (n2, n3, n4, n5):
    for U in (UX, UY, UZ):
        n.restrain(U)

# Loads
n1.add_load(0,  200000.0)
n1.add_load(1, -800000.0)
n1.add_load(2, -600000.0)

# Materials and Sections
STEEL_1 = SteelMaterial(
    material_id = "A36",
    nu = 0.30,
    E = 200000,        #MPa
)
SECTION_1 = WSection(
    section_id = "TRUSS-1", 
    shape_type = "W",
    area = 20000,     #mm^2
)
SECTION_2 = WSection(
    section_id = "TRUSS-2", 
    shape_type = "W",
    area = 30000,     #mm^2
)
SECTION_3 = WSection(
    section_id = "TRUSS-3", 
    shape_type = "W",
    area = 40000,     #mm^2
)
SECTION_4 = WSection(
    section_id = "TRUSS-4", 
    shape_type = "W",
    area = 30000,     #mm^2
)

# Elements
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

# Assembly
for n in (n1, n2, n3, n4, n5):
    MODEL_1.add_node(n)

for truss in (TRUSS_1, TRUSS_2, TRUSS_3, TRUSS_4):
    MODEL_1.add_element(truss)

MODEL_1.solve()

# Results
print("\nNode 1 Displacements:")
for dof, val in n1.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")

print("\nNode 2 Reactions:")
for reactions, val in n2.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 3 Reactions:")
for reactions, val in n3.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 4 Reactions:")
for reactions, val in n4.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 5 Reactions:")
for reactions, val in n5.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")



