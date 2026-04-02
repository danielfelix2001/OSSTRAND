from src.core.geometry.node import Node
from src.core.elements.frame import Frame
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
from src.visualization.viewer import SolutionStateViewer

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
# Portal Frame 1
N1 = Node("N1",    0.0,    0.0,     0.0)
N2 = Node("N2", 5000.0,    0.0,     0.0)
N3 = Node("N3",    0.0, 3000.0,     0.0)
N4 = Node("N4", 5000.0, 3000.0,     0.0)

# Portal Frame 2
N5 = Node("N5",    0.0,    0.0, -4000.0)
N6 = Node("N6", 5000.0,    0.0, -4000.0)
N7 = Node("N7",    0.0, 3000.0, -4000.0)
N8 = Node("N8", 5000.0, 3000.0, -4000.0)

# Pin restraints
for node in [N1, N2, N5, N6]:
    for dof in [gv.UX, gv.UY, gv.UZ]:
        node.restrain(dof)

# Container        
NODES = [N1, N2, N3, N4, N5, N6, N7, N8]

# --------------------------------
# MATERIAL AND SECTION
# --------------------------------
A36_STEEL = Material(
    material_id="A36_STEEL",
    E = 200000,  # MPa
    nu = 0.3,
    # gamma = 7850 * 9.81 * 10**(-9) # N/mm^3
)
FRAME_SECTION = Section(
    section_id = "W200x15", 
    area = 1910,    # mm^2
    Ixx = 12.8e+06, # mm^4
    Iyy = 0.87e+06, # mm^4
    J = 17.7e+03    # mm^4
)
TRUSS_SECTION = Section(
    section_id = "TRUSS-1",
    area = 500,    # mm^2
)

# --------------------------------
# ELEMENTS
# --------------------------------
# Portal Frame 1
E1 = Frame("E1", N1, N3, A36_STEEL, FRAME_SECTION)
E2 = Frame("E2", N3, N4, A36_STEEL, FRAME_SECTION)
E3 = Frame("E3", N4, N2, A36_STEEL, FRAME_SECTION)

# Portal Frame 2
E4 = Frame("E4", N5, N7, A36_STEEL, FRAME_SECTION)
E5 = Frame("E5", N7, N8, A36_STEEL, FRAME_SECTION)
E6 = Frame("E6", N8, N6, A36_STEEL, FRAME_SECTION)

# Trusses
E7 = Truss("E7", N3, N7, A36_STEEL, TRUSS_SECTION)
E8 = Truss("E8", N3, N5, A36_STEEL, TRUSS_SECTION)
E9 = Truss("E9", N4, N8, A36_STEEL, TRUSS_SECTION)
E10= Truss("E10",N4, N6, A36_STEEL, TRUSS_SECTION)

# Container
ELEMENTS = [E1, E2, E3, E4, E5, E6, E7, E8, E9, E10]

# --------------------------------
# LOADS AND LOAD COMBINATIONS
# --------------------------------
N3_UZ = NodalLoad(
    id = "N3_UZ",
    node = N3,
    dof = gv.UZ,
    magnitude = -5000.0
)
N4_UZ = NodalLoad(
    id = "N4_UZ",
    node = N4,
    dof = gv.UZ,
    magnitude = -5000.0
)

DEAD_LOAD = LoadCase(
    name = "Dead_Load"
)
DEAD_LOAD.add_nodal_load(N3_UZ)
DEAD_LOAD.add_nodal_load(N4_UZ)

LC1 = LoadCombination(
    name = "LC1",
    loadCaseAndFactors = {
        DEAD_LOAD: 1.0
    }
)

# --------------------------------
# MODEL ASSEMBLY
# --------------------------------
MODEL = Model()

for node in NODES:
    MODEL.add_node(node)

for element in ELEMENTS:
    MODEL.add_element(element)
   
Preprocess(MODEL)   
solution = LinearStaticSolve(MODEL, LC1)

# --------------------------------
# RESULTS
# --------------------------------
# print("\nSolution Displacement Vector")
# print(f"{solution.displacements}")

# print("\nNode 3 Displacements:")
# for disp in gv.GLOBAL_DISP_DOFS:
#     print(f"{names.DOF[disp]}: {solution.node_displacement(N3.id, disp):.3e}")

# print("\nElement 1 Local-End Forces, Node i:")
# for force in gv.LOCAL_FORCES_FRAME:
#     print(f"{names.LOCAL_REACTION_FRAME[force]}: {solution.local_element_end_force("E1", gv.NODE_i, force):.3e}")

# print("\nElement 1 Local-End Forces, Node j:")
# for force in gv.LOCAL_FORCES_FRAME:
#     print(f"{names.LOCAL_REACTION_FRAME[force]}: {solution.local_element_end_force("E1", gv.NODE_j, force):.3e}")

# --------------------------------
# SHOW
# --------------------------------
viewer = SolutionStateViewer(solution)
viewer.show()