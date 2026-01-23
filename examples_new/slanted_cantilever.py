from src.model.geometry.node import Node
from src.model.elements.frame import Frame
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
N1 = Node(1,    0.0,    0.0,    0.0)
N2 = Node(2, 5000.0, 4000.0, 3000.0)

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
    element = E1, 
    local = True,
    wx = 5.0,
    wy = 6.0,
    wz = 7.0
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
SLANT_CANTILEVER = Model()
SLANT_CANTILEVER.add_node(N1)
SLANT_CANTILEVER.add_node(N2)
SLANT_CANTILEVER.add_element(E1)

# Solve
SLANT_CANTILEVER.preprocess()
SLANT_CANTILEVER.linear_static_solve(LC1)

# --------------------------------
# RESULTS
# --------------------------------
print("\nFree end displacements:")
print(f"UX: {N2.DISPLACEMENT(gv.UX)}")
print(f"UY: {N2.DISPLACEMENT(gv.UY)}")
print(f"UZ: {N2.DISPLACEMENT(gv.UZ)}")

