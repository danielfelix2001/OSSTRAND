from src.core.geometry.node import Node
from src.core.elements.beam import Beam
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
N1 = Node("N1", 0.0, 0.0, 0.0)
N2 = Node("N2", 5000.0, 0.0, 0.0)

# Cantilever restraints
for dof in [gv.UY, gv.UZ, gv.MY, gv.MZ]:
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
    id = "UDL_1",
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

Preprocess(MODEL_1)
solution = LinearStaticSolve(MODEL_1, LC1)

# --------------------------------
# RESULTS
# --------------------------------
# print("Node 1 Reactions:")
# for idx in range(6):
#     print(f"{names.GLOBAL_REACTION[idx]}: {solution.node_reaction(N1.id, idx):.4}")

# print("\nNode 2 Displacements:")
# for idx in range(6):
#     print(f"{names.DOF[idx]}: {solution.node_displacement(N2.id, idx):.4}")

# --------------------------------
# SHOW
# --------------------------------
viewer = ModelViewer(MODEL_1)
viewer.show()