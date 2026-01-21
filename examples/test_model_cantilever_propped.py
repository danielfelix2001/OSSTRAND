from source.model.nodes import Node
from source.model.lineElements.frame import Frame
from source.model.loads.loadCombo import LoadCombination, LoadCase, NodalLoad, UDL, SlfWgt, PntLd
from source.model.materials import Material
from source.model.sections import Section
from source.model.model import Model
from source.model.functions import DOF_NAMES, GLOBAL_REACTION_NAMES, LOCAL_REACTION_NAMES, LOCAL_ELEMENT_REACTION_NAMES
import numpy as np

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

# units in N, mm
N1 = Node(1,     0.0,     0.0,     0.0)
N2 = Node(2,  5000.0,     0.0,     0.0)
N3 = Node(3,  5000.0, -3000.0,     0.0)

# Fixed Restraints
for dof in [UX, UY, UZ, RX, RY, RZ]:
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
E2.release(node="i", dof=RX)
E2.release(node="i", dof=RY)
E2.release(node="i", dof=RZ)

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
    dof = UY,
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
MODEL.solve_load_combo(LC1)


# --------------------------------
# RESULTS
# --------------------------------
print("\nNode 2 displacements:")
for dof, val in N2.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")

