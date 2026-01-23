from src.model.geometry.node import Node
from src.model.elements.frame import Frame
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
# Portal Frame 1
N1 = Node(1,    0.0,    0.0,     0.0)
N2 = Node(2, 5000.0,    0.0,     0.0)
N3 = Node(3,    0.0, 3000.0,     0.0)
N4 = Node(4, 5000.0, 3000.0,     0.0)

# Portal Frame 2
N5 = Node(5,    0.0,    0.0, -4000.0)
N6 = Node(6, 5000.0,    0.0, -4000.0)
N7 = Node(7,    0.0, 3000.0, -4000.0)
N8 = Node(8, 5000.0, 3000.0, -4000.0)

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
PORTAL_FRAME_SECTION = Section(
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
E1 = Frame("E1", N1, N3, A36_STEEL, PORTAL_FRAME_SECTION)
E2 = Frame("E2", N3, N4, A36_STEEL, PORTAL_FRAME_SECTION)
E3 = Frame("E3", N4, N2, A36_STEEL, PORTAL_FRAME_SECTION)

# Portal Frame 1
E4 = Frame("E4", N5, N7, A36_STEEL, PORTAL_FRAME_SECTION)
E5 = Frame("E5", N7, N8, A36_STEEL, PORTAL_FRAME_SECTION)
E6 = Frame("E6", N8, N6, A36_STEEL, PORTAL_FRAME_SECTION)

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
    node = N3,
    dof = gv.UZ,
    magnitude = -5000.0
)
N4_UZ = NodalLoad(
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
   
MODEL.preprocess()   
MODEL.linear_static_solve(LC1)

# --------------------------------
# RESULTS
# --------------------------------
print("\nNode 3 Displacements:")
for dof, val in N3.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")
