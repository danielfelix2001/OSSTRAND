from source.model.nodes import Node
from source.model.lineElements.frame import Frame
from source.model.fixedEndForces.fefs import UDL
from source.model.materials import Material
from source.model.sections import Section
from source.model.model import Model
from source.model.functions import DOF_NAMES, GLOBAL_REACTION_NAMES, LOCAL_REACTION_NAMES, LOCAL_ELEMENT_REACTION_NAMES
import numpy as np

PI = 3.14159265
# Force DOFs
NX, VY, VZ = 0, 1, 2
TX, MY, MZ = 3, 4, 5
FORCE_DOFS = [NX, VY, VZ, TX, MY, MZ]

# Displacement DOFs
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
N1 = Node(1,    0.0,    0.0,    0.0)
N2 = Node(2, 5000.0, 4000.0, 3000.0)

# Fixed Restraints
for dof in [UX, UY, UZ, RX, RY, RZ]:
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
    section=FRAME_SECTION
)

# --------------------------------
# LOADS
# --------------------------------
E1.add_load(
    UDL(
        local=False,
        wx = 5.0,
        wy = 6.0,
        wz = 7.0
    )
)

# --------------------------------
# MODEL ASSEMBLY
# --------------------------------
SLANT_CANTILEVER = Model()
SLANT_CANTILEVER.add_node(N1)
SLANT_CANTILEVER.add_node(N2)
SLANT_CANTILEVER.add_element(E1)
SLANT_CANTILEVER.solve()

# --------------------------------
# RESULTS
# --------------------------------
print("\nFree end displacements:")
print(f"UX: {N2.DISPLACEMENT(UX)}")
print(f"UY: {N2.DISPLACEMENT(UY)}")
print(f"UZ: {N2.DISPLACEMENT(UZ)}")

