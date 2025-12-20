from src.model.nodes import Node
from src.model.lineElements.beam import Beam
from src.model.fixedEndForces.fefs import UDL
from src.model.materials import SteelMaterial
from src.model.sections import SteelSection
from src.model.model import Model
from src.model.labels import DOF_NAMES, REACTION_NAMES

"""
Global xyz system
x to right
y up
z backward
"""

MODEL_1 = Model()

n1 = Node(1, 0.0, 0.0, 0.0)
n2 = Node(2, 5000.0, 0.0, 0.0)

# Cantilever restraints
n1.restrain(1) #y
n1.restrain(2) #z
n1.restrain(4) #ry
n1.restrain(5) #rz

STEEL_1 = SteelMaterial(
    material_id = "A36",
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

BEAM_1 = Beam(
    element_id = "1,2",
    node_i = n1,
    node_j = n2,
    material = STEEL_1,
    section = SECTION_1,
    roll_radians = 0.0
)

# Loads
n2.add_load(1, -1000.0) # N in global Y
BEAM_1.add_load(UDL(qy = -1.0)) # N/mm, local y

# Assembly
MODEL_1.add_node(n1)
MODEL_1.add_node(n2)
MODEL_1.add_element(BEAM_1)

MODEL_1.solve()

# Results
print("Node 1 Reactions:")
for reactions, val in n1.reactions.items():
    print(f"{REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 2 Displacements:")
for dof, val in n2.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")


