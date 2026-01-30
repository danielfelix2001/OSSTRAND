from src.core.geometry.node import Node
from src.core.elements.frame import Frame
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
N1 = Node("N1",    0.0,    0.0,    0.0)
N2 = Node("N2", 5000.0, 4000.0, 3000.0)

for dof in gv.GLOBAL_DISP_DOFS:
    N1.restrain(dof)

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

# --------------------------------
# ELEMENTS
# --------------------------------
E1 = Frame(
    element_id="E1",
    node_i=N1,
    node_j=N2,
    material=A36_STEEL,
    section=FRAME_SECTION,
    roll_radians = 0.0
)

# --------------------------------
# LOADS AND LOAD COMBINATIONS
# --------------------------------
UDL_Wxyz = UDL(
    id = "UDL_xyz",
    element = E1, 
    local = True,
    wx = 0.5,
    wy = 0.6,
    wz = 0.7
)

DEAD_LOAD = LoadCase(
    name = "Dead_Load"
)
DEAD_LOAD.add_element_load(UDL_Wxyz)

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
MODEL.add_node(N1)
MODEL.add_node(N2)
MODEL.add_element(E1)

# Solve
Preprocess(MODEL)
solution = LinearStaticSolve(MODEL, LC1)

# --------------------------------
# RESULTS
# --------------------------------
print("\nFree end displacements:")
for disp in gv.GLOBAL_DISP_DOFS:
    print(f"{names.DOF[disp]}: {solution.node_displacement(N2.id, disp):.3e}")

