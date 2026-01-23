from src.model.geometry.node import Node
from src.model.elements.truss import Truss
from src.model.materials.base_material import Material
from src.model.sections.base_section import Section
from src.model.model import Model

from src.model.loads.load_combo import LoadCombination
from src.model.loads.load_case import LoadCase
from src.model.loads.nodal_load import NodalLoad
from src.model.loads.element_load import UDL, SlfWgt, PntLd

from src.utils.helpers import DOF_NAMES, GLOBAL_REACTION_NAMES, LOCAL_REACTION_NAMES, LOCAL_ELEMENT_REACTION_NAMES
from src.utils import global_variables as gv
import math

"""
Global xyz system
x: right
y: up
z: backward

units in N, mm
"""

# --------------------------------
# NODES AND RESTRAINTS
# --------------------------------
N1 = Node(1,     0.0,     0.0,     0.0)
N2 = Node(2, -2000.0, -8000.0,  4000.0)
N3 = Node(3,  6000.0, -8000.0,  4000.0)
N4 = Node(4,  6000.0, -8000.0, -2000.0)
N5 = Node(5, -2000.0, -8000.0, -2000.0)

TRUSS_DOFS = (gv.UX, gv.UY, gv.UZ)
for N in (N2, N3, N4, N5):
    for dof in TRUSS_DOFS:
        N.restrain(dof)

# --------------------------------
# MATERIAL AND SECTION
# --------------------------------
STEEL_1 = Material(
    material_id = "A36",
    nu = 0.30,
    E = 200000,        #MPa
)
SECTION_1 = Section(
    section_id = "TRUSS-1", 
    area = 20000,     #mm^2
)
SECTION_2 = Section(
    section_id = "TRUSS-2", 
    area = 30000,     #mm^2
)
SECTION_3 = Section(
    section_id = "TRUSS-3", 
    area = 40000,     #mm^2
)
SECTION_4 = Section(
    section_id = "TRUSS-4", 
    area = 30000,     #mm^2
)

# --------------------------------
# ELEMENTS
# --------------------------------
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

# --------------------------------
# LOADS AND LOAD COMBINATIONS
# --------------------------------
N1_FX = NodalLoad(
    node = N1,
    dof = gv.FX,
    magnitude = 200000.0
)
N1_FY = NodalLoad(
    node = N1,
    dof = gv.FY,
    magnitude = -800000.0
)
N1_FZ = NodalLoad(
    node = N1,
    dof = gv.FZ,
    magnitude = -600000.0
)

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

# --------------------------------
# MODEL ASSEMBLY
# --------------------------------
MODEL_SPACE_TRUSS = Model()
for N in (N1, N2, N3, N4, N5):
    MODEL_SPACE_TRUSS.add_node(N)
for truss in (TRUSS_1, TRUSS_2, TRUSS_3, TRUSS_4):
    MODEL_SPACE_TRUSS.add_element(truss)

# Solve
MODEL_SPACE_TRUSS.preprocess()
MODEL_SPACE_TRUSS.linear_static_solve(LC1)

# --------------------------------
# RESULTS
# --------------------------------
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