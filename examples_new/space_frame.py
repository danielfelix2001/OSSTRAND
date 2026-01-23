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

units in kip, inch
"""

# --------------------------------
# NODES AND RESTRAINTS
# --------------------------------
NODE_1 = Node(1,    0.0,    0.0,    0.0)
NODE_2 = Node(2, -240.0,    0.0,    0.0)
NODE_3 = Node(3,    0.0, -240.0,    0.0)
NODE_4 = Node(4,    0.0,    0.0, -240.0)

for n in (NODE_2, NODE_3, NODE_4):
    for dof in gv.GLOBAL_DISP_DOFS:
        n.restrain(dof)

# --------------------------------
# MATERIAL AND SECTION
# --------------------------------
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

# --------------------------------
# ELEMENTS
# --------------------------------
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
    roll_radians = 90*math.pi/180  
)
ELEMENT_3 = Frame(
    element_id = "4,1",
    node_i = NODE_4,
    node_j = NODE_1,
    material = MATERIAL_1,
    section = SECTION_1,
    roll_radians = 30*math.pi/180 
)

# --------------------------------
# LOADS AND LOAD COMBINATIONS
# --------------------------------
N1_MX = NodalLoad( # Tx, -150 kip-ft
    node = NODE_1,
    dof = gv.MX,
    magnitude = -1800.0
)
N1_MZ = NodalLoad( # Mz, 150 kip-ft
    node = NODE_1,
    dof = gv.MZ,
    magnitude = 1800.0
)
E1_localUDL_Y = UDL( # qy, 3 kip/ft
    element = ELEMENT_1,
    local = True, 
    wx = 0.0, 
    wy = -0.25, 
    wz = 0.0
)

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

# --------------------------------
# MODEL ASSEMBLY
# --------------------------------
MODEL_SPACE_FRAME = Model()
for node in (NODE_1, NODE_2, NODE_3, NODE_4):
    MODEL_SPACE_FRAME.add_node(node)
for element in (ELEMENT_1, ELEMENT_2, ELEMENT_3):
    MODEL_SPACE_FRAME.add_element(element)

MODEL_SPACE_FRAME.preprocess()

# --------------------------------
# RESULTS
# --------------------------------
for LC in LCs:
    print(f"\nNode 1 Displacements for {LC.name}:")
    MODEL_SPACE_FRAME.linear_static_solve(LC)

    for disp in gv.GLOBAL_DISP_DOFS:
        print(f"{DOF_NAMES[disp]}: {NODE_1.DISPLACEMENT(disp):.3e}")