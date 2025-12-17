from src.model.nodes import Node
from src.model.lineElements.truss import Truss
from src.model.materials import SteelMaterial
from src.model.sections import SteelSection
from src.model.model import Model
from src.model.dofs import DOF_NAMES, REACTION_NAMES
import numpy as np

"""
Store nodes
Store materials
Store sections
Store elements
Assign global DOF numbers
Assemble K and F
Call the solver
"""

"""
Global xyz system
x to right
y up
z backward
"""

MODEL_1 = Model()

n1 = Node(1, 0.0, 0.0, 0.0)
n2 = Node(2, 5000.0, 0.0, 0.0)

# Cantilever
n1.restrain(0) #x
n1.restrain(1) #y
n1.restrain(2) #z

# Loads
n2.add_load(0, -1000.0)

# --------------------------------
# MATERIAL AND SECTION
# should come from a database in the future
# --------------------------------
STEEL_1 = SteelMaterial(
    material_id = "A36",
    density_KG_PER_M3 = 7850,   #kg/m^3
    nu = 0.30,
    E = 200000,        #MPa
    fy = 250,          #MPa
    fu = 400           #MPa
)
SECTION_1 = SteelSection(
    section_id = "W200x15", #W8x10 in english units
    shape_type = "W",
    area = 1910,     #mm^2
    Ixx = 12.8e+06,  #mm^4
    Iyy = 0.87e+06,  #mm^4
    J = 17.7e+03,    #mm^4
    Sx = 128e+03,    #mm^3
    Sy = 17.4e+03,   #mm^3
    Zx = 145e+03,    #mm^3
    Zy = 27.2e+03    #mm^3
)

# --------------------------------
# ELEMENTS
# --------------------------------
TRUSS_1 = Truss(
    element_id = "1,2",
    node_i = n1,
    node_j = n2,
    material = STEEL_1,
    section = SECTION_1
)

# Add UDL to element
# BEAM_1.add_udl(qy = -1.0) # N/mm, local y

# --------------------------------
# ASSEMBLY AND SOLVING
# --------------------------------
MODEL_1.add_node(n1)
MODEL_1.add_node(n2)
MODEL_1.add_element(TRUSS_1)
MODEL_1.collect_node_dofs()
MODEL_1.assign_dofs()

MODEL_1.assemble_stiffness()
MODEL_1.assemble_loads()
MODEL_1.assemble_fixed_end_forces()

MODEL_1.solve()
MODEL_1.store_displacements()
MODEL_1.store_reactions()

# --------------------------------
# RESULTS
# --------------------------------
print("Node 1 Reactions:")
for reactions, val in n1.reactions.items():
    print(f"{REACTION_NAMES[reactions]} = {val:.4e}")

print("\nNode 2 Displacements:")
for dof, val in n2.displacements.items():
    print(f"{DOF_NAMES[dof]} = {val:.4e}")


