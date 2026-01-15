from source.model.nodes import Node
from source.model.lineElements.beam import Beam
from source.model.fixedEndForces.fefs import UDL
from source.model.materials import SteelMaterial
from source.model.sections import WSection
from source.model.model import Model
from source.model.functions import DOF_NAMES, GLOBAL_REACTION_NAMES, LOCAL_REACTION_NAMES
UX, UY, UZ = 0, 1, 2
RX, RY, RZ = 3, 4, 5

"""
Global xyz system
x to right
y up
z backward
"""

MODEL_1 = Model()

n1 = Node(1, 0.0, 0.0, 0.0)
n2 = Node(1, 5000.0, 0.0, 0.0)

# Cantilever restraints
n1.restrain(UY) 
n1.restrain(UZ) 
n1.restrain(RY) 
n1.restrain(RZ) 

STEEL_1 = SteelMaterial(
    material_id = "A36",
    nu = 0.30,
    E = 200000,        #MPa
    fy = 250,          #MPa
    fu = 400           #MPa
)
SECTION_1 = WSection(
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
# n2.add_load(1, -1000.0) # N in global Y
BEAM_1.add_load(UDL(local = True, wy = -1.0)) # N/mm, local y

# Assembly
MODEL_1.add_node(n1)
MODEL_1.add_node(n2)
MODEL_1.add_element(BEAM_1)

MODEL_1.solve()

# Results
print("Node 1 Reactions:")
for reactions, val in n1.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 2 Displacements:")
for dof, val in n2.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")

# Location along length L
x = 2500

# Internal Forces
print("\nInternal Forces at x = {2500}")
print(f"{LOCAL_REACTION_NAMES[UY]} = {MODEL_1.element["1,2"].Vy_internal(x):.4e}")
print(f"{LOCAL_REACTION_NAMES[UZ]} = {MODEL_1.element["1,2"].Vz_internal(x):.4e}")
print(f"{LOCAL_REACTION_NAMES[RY]} = {MODEL_1.element["1,2"].My_internal(x):.4e}")
print(f"{LOCAL_REACTION_NAMES[RZ]} = {MODEL_1.element["1,2"].Mz_internal(x):.4e}")
