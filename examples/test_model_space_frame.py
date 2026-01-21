from source.model.nodes import Node
from source.model.lineElements.frame import Frame
from source.model.loads.loadCombo import LoadCombination, LoadCase, NodalLoad, UDL, SlfWgt, PntLd
from source.model.materials import Material
from source.model.sections import Section
from source.model.model import Model
from source.model.functions import DOF_NAMES, GLOBAL_REACTION_NAMES, LOCAL_REACTION_NAMES, LOCAL_ELEMENT_REACTION_NAMES

PI = 3.14159265

# Global Force DOFs
FX, FY, FZ = 0, 1, 2
MX, MY, MZ = 3, 4, 5
GLOBAL_FORCE_DOFS = [FX, FY, FZ, MX, MY, MZ]

# Local Force DOFs
Nx, Vy, Vz  = 0, 1, 2
Tx, My, Mz = 3, 4, 5
LOCAL_FORCE_DOFS = [Nx, Vy, Vz, Tx, My, Mz]

# Global Displacement DOFs
UX, UY, UZ = 0, 1, 2
RX, RY, RZ = 3, 4, 5
DISP_DOFS = [UX, UY, UZ, RX, RY, RZ]

"""
Global xyz system
x to right
y up
z backward
"""

# units in kip, inch
MODEL_SPACE_FRAME = Model()

NODE_1 = Node(1,    0.0,    0.0,    0.0)
NODE_2 = Node(2, -240.0,    0.0,    0.0)
NODE_3 = Node(3,    0.0, -240.0,    0.0)
NODE_4 = Node(4,    0.0,    0.0, -240.0)

for n in (NODE_2, NODE_3, NODE_4):
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
    node_i = NODE_2,
    node_j = NODE_1,
    material = MATERIAL_1,
    section = SECTION_1,
    roll_radians = 0    
)
ELEMENT_2 = Frame(
    element_id = "3,1",
    node_i = NODE_3,
    node_j = NODE_1,
    material = MATERIAL_1,
    section = SECTION_1,
    roll_radians = 90*PI/180  
)
ELEMENT_3 = Frame(
    element_id = "4,1",
    node_i = NODE_4,
    node_j = NODE_1,
    material = MATERIAL_1,
    section = SECTION_1,
    roll_radians = 30*PI/180 
)

# Assembly
for node in (NODE_1, NODE_2, NODE_3, NODE_4):
    MODEL_SPACE_FRAME.add_node(node)
for element in (ELEMENT_1, ELEMENT_2, ELEMENT_3):
    MODEL_SPACE_FRAME.add_element(element)

# Loads 
N1_MX = NodalLoad( # Tx, -150 kip-ft
    node = NODE_1,
    dof = MX,
    magnitude = -1800.0
)
N1_MZ = NodalLoad( # Mz, 150 kip-ft
    node = NODE_1,
    dof = MZ,
    magnitude = 1800.0
)
E1_localUDL_Y = UDL( # qy, 3 kip/ft
    element = ELEMENT_1,
    local = True, 
    wx = 0.0, 
    wy = -0.25, 
    wz = 0.0
)

# Load Cases and Combinations
DEAD_LOAD = LoadCase(
    name = "Dead_Load"
)
LIVE_LOAD = LoadCase(
    name = "Live_Load"
)

DEAD_LOAD.add_nodal_load(N1_MX)
DEAD_LOAD.add_nodal_load(N1_MZ)
DEAD_LOAD.add_element_load(E1_localUDL_Y)

LIVE_LOAD.add_nodal_load(N1_MX)
LIVE_LOAD.add_nodal_load(N1_MZ)
LIVE_LOAD.add_element_load(E1_localUDL_Y)

LC1 = LoadCombination(
    name = "LC1",
    loadCaseAndFactors = {
        DEAD_LOAD: 1.4
    }
)
LC2 = LoadCombination(
    name = "LC2",
    loadCaseAndFactors = {
        DEAD_LOAD: 1.2,
        LIVE_LOAD: 1.6
    }
)
LCs = [LC1, LC2]

# Solve
MODEL_SPACE_FRAME.preprocess()

# Results
for LC in LCs:
    print(f"\nNode 1 Displacements for {LC.name}:")
    MODEL_SPACE_FRAME.solve_load_combo(LC)

    for disp in DISP_DOFS:
        print(f"{DOF_NAMES[disp]}: {NODE_1.DISPLACEMENT(disp):.3e}")

# print("\nElement 1 End Forces:")
# print(f"{ELEMENT_1.END_FORCES(node_label="i", local=True)}")
# print(f"{ELEMENT_1.END_FORCES(node_label="i", local=False)}")

# print("\nNode 2 Reactions:")
# for reactions, val in n2.reactions.items(): 
#     print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

# print("\nNode 3 Reactions:")
# for reactions, val in n3.reactions.items(): 
#     print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

# print("\nNode 4 Reactions:")
# for reactions, val in n4.reactions.items(): 
#     print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

# print("\nElement 1 End Forces")
# forces = ["Nx_i", "Vy_i", "Vz_i", "Tx_i", "My_i", "Mz_i",
#                   "Nx_j", "Vy_j", "Vz_j", "Tx_j", "My_j", "Mz_j"]
# for index, force in enumerate(forces):
#     print(f"{LOCAL_ELEMENT_REACTION_NAMES[index]} = {getattr(ELEMENT_1, force):4f}")

# print("\nElement 1 Internal Forces")
# location = [0.0, 120.0, 240.0] # start, midspan, end

# internalForces = ["Nx_internal", "Vy_internal", "Vz_internal", "Tx_internal", "My_internal", "Mz_internal"]
# for x in location:
#     for index, force in enumerate(internalForces):
#         print(f"{LOCAL_REACTION_NAMES[index]}({x})  = {getattr(ELEMENT_1, force)(x):4f}")
#     print("")

# print("\nElement 1 Internal Stresses")
# y = 100.0
# z = 50.0
# for x in location:
#     print(f"Axial Stress at {x}                 = {ELEMENT_1.axial_stress(x):4f}")
#     print(f"Bending Stress about y at {x}       = {ELEMENT_1.bending_stress_about_y(x, z):4f}")
#     print(f"Bending Stress about z at {x}       = {ELEMENT_1.bending_stress_about_z(x, y):4f}")
#     print(f"Normal Stress at {x}                = {ELEMENT_1.normal_stress(x, y, z):4f}")
#     print("")

# print(ELEMENT_2.get_end_forces(node_label = "j", local = False))