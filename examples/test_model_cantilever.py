from source.model.nodes import Node
from source.model.lineElements.beam import Beam
from source.model.loads.loadCombo import LoadCombination, LoadCase, NodalLoad, UDL
from source.model.materials import Material
from source.model.sections import Section
from source.model.model import Model
from source.model.functions import DOF_NAMES, GLOBAL_REACTION_NAMES, LOCAL_REACTION_NAMES

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

# units in mm
N1 = Node(1, 0.0, 0.0, 0.0)
N2 = Node(2, 5000.0, 0.0, 0.0)

# Cantilever restraints
for dof in [UY, UZ, MY, MZ]:
    N1.restrain(dof)

# --------------------------------
# MATERIAL AND SECTION
# --------------------------------
STEEL_1 = Material(
    material_id = "A36_STEEL",
    nu = 0.30,
    E = 200000        #MPa
)
SECTION_1 = Section(
    section_id = "W200x15", #W8x10 in english units
    shape_type = "W",
    area = 1910,     #mm^2
    Ixx = 12.8e+06,  #mm^4
    Iyy = 0.87e+06,  #mm^4
    J = 17.7e+03,    #mm^4
)

# --------------------------------
# ELEMENTS
# --------------------------------
E1 = Beam(
    element_id = "E1",
    node_i = N1,
    node_j = N2,
    material = STEEL_1,
    section = SECTION_1,
    roll_radians = 0.0
)

# --------------------------------
# LOADS AND LOAD COMBINATIONS
# --------------------------------
UDL_wy = UDL(
    element = E1,
    local = True,
    wx =  0.0,
    wy = -1.0,
    wz =  0.0
)

DEAD_LOAD = LoadCase(
    name = "Dead_Load"
)
DEAD_LOAD.add_element_load(UDL_wy)

LC1 = LoadCombination(
    name = "LC1",
    loadCaseAndFactors = {
        DEAD_LOAD: 1.0
    }
)

# --------------------------------
# MODEL ASSEMBLY
# --------------------------------
MODEL_1 = Model()
MODEL_1.add_node(N1)
MODEL_1.add_node(N2)
MODEL_1.add_element(E1)

MODEL_1.preprocess()
MODEL_1.solve_load_combo(LC1)

# --------------------------------
# RESULTS
# --------------------------------

print("Node 1 Reactions:")
for reactions, val in N1.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 2 Displacements:")
for dof, val in N2.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")

# Location along length L
# x = 2500

# Internal Forces
# print("\nInternal Forces at x = {2500}")
# print(f"{LOCAL_REACTION_NAMES[UY]} = {MODEL_1.element["1,2"].Vy_internal(x):.4e}")
# print(f"{LOCAL_REACTION_NAMES[UZ]} = {MODEL_1.element["1,2"].Vz_internal(x):.4e}")
# print(f"{LOCAL_REACTION_NAMES[RY]} = {MODEL_1.element["1,2"].My_internal(x):.4e}")
# print(f"{LOCAL_REACTION_NAMES[RZ]} = {MODEL_1.element["1,2"].Mz_internal(x):.4e}")
