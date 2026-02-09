from src.core.geometry.node import Node
from src.core.elements.truss import Truss
from src.core.materials.base_material import Material
from src.core.sections.base_section import Section
from src.core.model import Model

from src.core.loads.load_combo import LoadCombination
from src.core.loads.load_case import LoadCase
from src.core.loads.nodal_load import NodalLoad
from src.core.loads.element_load import UDL, SelfWeight, PointLoad

from src.core.analysis.preprocessing import Preprocess
from src.core.analysis.linear_static import LinearStaticSolve
from src.core.results.solution_state import SolutionState
from src.visualization.viewer import ModelViewer

from src.utils import helpers as names 
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
N1 = Node("N1",     0.0,     0.0,     0.0)
N2 = Node("N2", -2000.0, -8000.0,  4000.0)
N3 = Node("N3",  6000.0, -8000.0,  4000.0)
N4 = Node("N4",  6000.0, -8000.0, -2000.0)
N5 = Node("N5", -2000.0, -8000.0, -2000.0)

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
    id = "N1_FX",
    node = N1,
    dof = gv.FX,
    magnitude = 200000.0
)
N1_FY = NodalLoad(
    id = "N1_FY",
    node = N1,
    dof = gv.FY,
    magnitude = -800000.0
)
N1_FZ = NodalLoad(
    id = "N1_FZ",
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
Preprocess(MODEL_SPACE_TRUSS)
solution = LinearStaticSolve(MODEL_SPACE_TRUSS, LC1)

# --------------------------------
# RESULTS
# --------------------------------
# print("\nNode 2 Displacements:")
# for disp in gv.GLOBAL_DISP_DOFS_TRUSS:
#     print(f"{names.DOF[disp]}: {solution.node_displacement(N1.id, disp):.3e}")

# print("\nNode 2 Reactions:")
# for reaction in gv.GLOBAL_FORCES_TRUSS:
#     print(f"{names.DOF[reaction]}: {solution.node_reaction(N2.id, reaction):.3e}")
# print("\nNode 3 Reactions:")
# for reaction in gv.GLOBAL_FORCES_TRUSS:
#     print(f"{names.DOF[reaction]}: {solution.node_reaction(N3.id, reaction):.3e}")
# print("\nNode 4 Reactions:")
# for reaction in gv.GLOBAL_FORCES_TRUSS:
#     print(f"{names.DOF[reaction]}: {solution.node_reaction(N4.id, reaction):.3e}")
# print("\nNode 5 Reactions:")
# for reaction in gv.GLOBAL_FORCES_TRUSS:
#     print(f"{names.DOF[reaction]}: {solution.node_reaction(N5.id, reaction):.3e}")

# --------------------------------
# SHOW
# --------------------------------
viewer = ModelViewer(MODEL_SPACE_TRUSS)
viewer.show()