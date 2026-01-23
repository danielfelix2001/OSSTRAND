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
N1 = Node(1,     0.0,     0.0,     0.0)
N2 = Node(2,  5000.0,     0.0,     0.0)
N3 = Node(3,  5000.0, -3000.0,     0.0)

for dof in gv.GLOBAL_DISP_DOFS:
    N1.restrain(dof)
    N3.restrain(dof)

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
    section=FRAME_SECTION
)
E2 = Frame(
    element_id="E2",
    node_i=N2,
    node_j=N3,
    material=A36_STEEL,
    section=FRAME_SECTION
)
E2.release(node="i", dof=gv.RX)
E2.release(node="i", dof=gv.RY)
E2.release(node="i", dof=gv.RZ)

# --------------------------------
# MODEL ASSEMBLY
# --------------------------------
MODEL = Model()
MODEL.add_node(N1)
MODEL.add_node(N2)
MODEL.add_node(N3)
MODEL.add_element(E1)
MODEL.add_element(E2)

# --------------------------------
# LOADS
# --------------------------------
N2_UY = NodalLoad(
    node = N2,
    dof = gv.UY,
    magnitude = -1000.0
)
DEAD_LOAD = LoadCase(
    name = "Dead_Load"
)
DEAD_LOAD.add_nodal_load(N2_UY)

LC1 = LoadCombination(
    name = "LC1",
    loadCaseAndFactors = {DEAD_LOAD: 1.0}
)

MODEL.preprocess()
MODEL.linear_static_solve(LC1)


# --------------------------------
# RESULTS
# --------------------------------
print("\nNode 2 displacements:")
for dof, val in N2.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")

