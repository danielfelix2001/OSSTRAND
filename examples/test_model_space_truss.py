from source.model.nodes import Node
from source.model.lineElements.truss import Truss
from source.model.loads.loadCombo import LoadCombination, LoadCase, NodalLoad
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

MODEL_1 = Model()

N1 = Node(1,     0.0,     0.0,     0.0)
N2 = Node(2, -2000.0, -8000.0,  4000.0)
N3 = Node(3,  6000.0, -8000.0,  4000.0)
N4 = Node(4,  6000.0, -8000.0, -2000.0)
N5 = Node(5, -2000.0, -8000.0, -2000.0)

for N in (N2, N3, N4, N5):
    for U in (UX, UY, UZ):
        N.restrain(U)

# Materials and Sections
STEEL_1 = Material(
    material_id = "A36",
    nu = 0.30,
    E = 200000,        #MPa
)
SECTION_1 = Section(
    section_id = "TRUSS-1", 
    shape_type = "W",
    area = 20000,     #mm^2
)
SECTION_2 = Section(
    section_id = "TRUSS-2", 
    shape_type = "W",
    area = 30000,     #mm^2
)
SECTION_3 = Section(
    section_id = "TRUSS-3", 
    shape_type = "W",
    area = 40000,     #mm^2
)
SECTION_4 = Section(
    section_id = "TRUSS-4", 
    shape_type = "W",
    area = 30000,     #mm^2
)

# Elements
TRUSS_1 = Truss(
    element_id = "1,2",
    node_i = N1,
    node_j = N2,
    material = STEEL_1,
    section = SECTION_1
)
TRUSS_2 = Truss(
    element_id = "1,3",
    node_i = N1,
    node_j = N3,
    material = STEEL_1,
    section = SECTION_2
)
TRUSS_3 = Truss(
    element_id = "1,4",
    node_i = N1,
    node_j = N4,
    material = STEEL_1,
    section = SECTION_3
)
TRUSS_4 = Truss(
    element_id = "1,5",
    node_i = N1,
    node_j = N5,
    material = STEEL_1,
    section = SECTION_4
)

# Assembly
for N in (N1, N2, N3, N4, N5):
    MODEL_1.add_node(N)

for truss in (TRUSS_1, TRUSS_2, TRUSS_3, TRUSS_4):
    MODEL_1.add_element(truss)

# Loads
N1_FX = NodalLoad(
    node = N1,
    dof = FX,
    magnitude = 200000.0
)
N1_FY = NodalLoad(
    node = N1,
    dof = FY,
    magnitude = -800000.0
)
N1_FZ = NodalLoad(
    node = N1,
    dof = FZ,
    magnitude = -600000.0
)

# Load Cases and Combinations
DEAD_LOAD = LoadCase(
    name = "Dead_Load"
)

DEAD_LOAD.add_nodal_load(N1_FX)
DEAD_LOAD.add_nodal_load(N1_FY)
DEAD_LOAD.add_nodal_load(N1_FZ)

LC1 = LoadCombination(
    name = "LC1",
    loadCaseAndFactors = {
        DEAD_LOAD: 1.0
    }
)

# Solve
MODEL_1.preprocess()
MODEL_1.solve_load_combo(LC1)

# Results
print("\nNode 1 Displacements:")
for dof, val in N1.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")

print("\nNode 2 Reactions:")
for reactions, val in N2.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 3 Reactions:")
for reactions, val in N3.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 4 Reactions:")
for reactions, val in N4.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 5 Reactions:")
for reactions, val in N5.reactions.items():
    print(f"{GLOBAL_REACTION_NAMES[reactions]} = {val:.4e}")