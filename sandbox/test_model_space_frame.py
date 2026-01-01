from source.model.nodes import Node
from source.model.lineElements.frame import Frame
from source.model.fixedEndForces.fefs import UDL
from source.model.materials import Material
from source.model.sections import Section
from source.model.model import Model
from source.model.functions import DOF_NAMES, GLOBAL_REACTION_NAMES, LOCAL_REACTION_NAMES, LOCAL_ELEMENT_REACTION_NAMES

PI = 3.14159265
FX, FY, FZ = 0, 1, 2
MX, MY, MZ = 3, 4, 5

"""
Global xyz system
x to right
y up
z backward
"""

# units in kip, inch
MODEL_SPACE_FRAME = Model()

n1 = Node(1,    0.0,    0.0,    0.0)
n2 = Node(2, -240.0,    0.0,    0.0)
n3 = Node(3,    0.0, -240.0,    0.0)
n4 = Node(4,    0.0,    0.0, -240.0)

for n in (n2, n3, n4):
    for idx in range (0,6): # restrain dofs 0 to 5
        n.restrain(idx)

MATERIAL_1 = Material(
    material_id = "MAT-1",
    E = 29000,       #ksi
    G = 11500        #ksi
)
SECTION_1 = Section(
    section_id = "SEC-1",
    area = 32.9,     #in^2
    Ixx = 716,       #in^4
    Iyy = 236,       #in^4
    J = 15.1        #in^4
)

ELEMENT_1 = Frame(
    element_id = "2,1",
    node_i = n2,
    node_j = n1,
    material = MATERIAL_1,
    section = SECTION_1,
    roll_radians = 0    
)
ELEMENT_2 = Frame(
    element_id = "3,1",
    node_i = n3,
    node_j = n1,
    material = MATERIAL_1,
    section = SECTION_1,
    roll_radians = 90*PI/180  
)
ELEMENT_3 = Frame(
    element_id = "4,1",
    node_i = n4,
    node_j = n1,
    material = MATERIAL_1,
    section = SECTION_1,
    roll_radians = 30*PI/180 
)

# Loads
n1.add_load(MX, -1800.0) # Mx, -150 kip-ft
n1.add_load(MZ,  1800.0) # Mz, 150 kip-ft
# ELEMENT_1.add_load(UDL(wy = -0.25)) # qy, 3 kip/ft

# Assembly
for node in (n1, n2, n3, n4):
    MODEL_SPACE_FRAME.add_node(node)
for element in (ELEMENT_1, ELEMENT_2, ELEMENT_3):
    MODEL_SPACE_FRAME.add_element(element)

MODEL_SPACE_FRAME.solve()

# Results

print("\nNode 1 Displacements:")
for d, val in n1.displacements.items():
    print(f"{DOF_NAMES[d]} = {val:.4e}")

print("\nNode 2 Reactions:")
for reactions, val in n2.reactions.items(): 
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 3 Reactions:")
for reactions, val in n3.reactions.items(): 
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 4 Reactions:")
for reactions, val in n4.reactions.items(): 
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nElement 1 End Forces")
forces = ["Nx_i", "Vy_i", "Vz_i", "Tx_i", "My_i", "Mz_i",
                  "Nx_j", "Vy_j", "Vz_j", "Tx_j", "My_j", "Mz_j"]
for index, force in enumerate(forces):
    print(f"{LOCAL_ELEMENT_REACTION_NAMES[index]} = {getattr(ELEMENT_1, force):4f}")

print("\nElement 1 Internal Forces")
location = [0.0, 120.0, 240.0] # start, midspan, end

internalForces = ["Nx_internal", "Vy_internal", "Vz_internal", "Tx_internal", "My_internal", "Mz_internal"]
for x in location:
    for index, force in enumerate(internalForces):
        print(f"{LOCAL_REACTION_NAMES[index]}({x})  = {getattr(ELEMENT_1, force)(x):4f}")
    print("")

print("\nElement 1 Internal Stresses")
y = 100.0
z = 50.0
for x in location:
    print(f"Axial Stress at {x}                 = {ELEMENT_1.axial_stress(x):4f}")
    print(f"Bending Stress about y at {x}       = {ELEMENT_1.bending_stress_about_y(x, z):4f}")
    print(f"Bending Stress about z at {x}       = {ELEMENT_1.bending_stress_about_z(x, y):4f}")
    print(f"Normal Stress at {x}                = {ELEMENT_1.normal_stress(x, y, z):4f}")
    print("")
