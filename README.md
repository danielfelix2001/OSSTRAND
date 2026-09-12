# OSSTRAND

Open-Source STRuctural ANalysis and Design software implemented in Python.

OSSTRAND is a lightweight structural analysis toolkit for linear static analysis of 2D/3D frame and truss systems. It includes:

- node and element model definition
- material and section property objects
- stiffness assembly and DOF assignment
- static linear solver for nodal and element loads
- load combinations and load cases
- basic visualization of the deformed structure

The codebase is currently organized as an educational and research-oriented structural analysis package rather than a full production engineering suite.

## Project status

The active implementation is under the `src/` package and is the path used by the examples. The project currently supports:

- `Node` objects with restrained DOFs
- `Frame`, `Beam`, and `Truss` elements
- `Material` and `Section` definitions
- `NodalLoad`, `UDL`, `SelfWeight`, and `PointLoad`
- `LoadCase` and `LoadCombination`
- `Preprocess()` and `LinearStaticSolve()`
- `SolutionState` result extraction
- PyVista-based structural visualization

## Repository structure

```text
OSSTRAND/
├── README.md
├── requirements.txt
├── examples_new/
│   ├── cantilever.py
│   ├── fixed_ended_beam.py
│   ├── portal_frame.py
│   ├── propped_cantilever.py
│   ├── slanted_cantilever.py
│   ├── space_frame.py
│   └── space_truss.py
├── source/
│   └── model/
│       ├── fixedEndForces/
│       └── lineElements/
└── src/
    ├── __init__.py
    ├── core/
    │   ├── analysis/
    │   │   ├── linear_static.py
    │   │   └── preprocessing.py
    │   ├── elements/
    │   │   ├── beam.py
    │   │   ├── frame.py
    │   │   └── truss.py
    │   ├── geometry/
    │   │   ├── base_element.py
    │   │   └── node.py
    │   ├── loads/
    │   │   ├── element_load.py
    │   │   ├── load_case.py
    │   │   ├── load_combo.py
    │   │   └── nodal_load.py
    │   ├── materials/
    │   │   └── base_material.py
    │   ├── model.py
    │   ├── results/
    │   │   └── solution_state.py
    │   └── sections/
    │       └── base_section.py
    ├── utils/
    │   ├── exceptions.py
    │   ├── global_variables.py
    │   └── helpers.py
    └── visualization/
        └── viewer.py
```

## Installation

1. Create a virtual environment (recommended)

```bash
python -m venv .venv
```

2. Activate it

On Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
```

On macOS/Linux:

```bash
source .venv/bin/activate
```

3. Install dependencies

```bash
pip install -r requirements.txt
```

The project dependencies include `numpy`, `pyvista`, and standard scientific plotting/tooling packages from `requirements.txt`.

## Core modeling workflow

The main workflow is:

1. Create nodes with coordinates
2. Define material and section properties
3. Create elements connecting nodes
4. Apply restraints to the nodes
5. Assemble a `Model`
6. Add load cases and combinations
7. Call `Preprocess(model)`
8. Run `LinearStaticSolve(model, load_combo)`
9. Query the `SolutionState` for displacements, reactions, and internal forces

This is the pattern used in the example scripts under `examples_new/`.

## Main classes and responsibilities

### Model

Defined in `src/core/model.py`.

The `Model` class stores:

- node registry (`self.node`)
- element registry (`self.element`)
- DOF metadata (`ndof`, `restrained_dofs`, `free_dofs`)
- assembled global stiffness matrix (`K_full`)
- force vector (`F_full`)
- displacement vector (`D_full`)
- reaction vector (`reactions`)
- preprocessing state flag (`preprocessed`)

It exposes `add_node()` and `add_element()` for assembly.

### Node

Defined in `src/core/geometry/node.py`.

A `Node` stores:

- `id`
- coordinates (`x`, `y`, `z`)
- `dofs` mapping of local DOF keys to global DOF indices
- `restraints` dictionary describing which DOFs are fixed

The `restrain()` method marks a DOF as restrained.

### Element base class

Defined in `src/core/geometry/base_element.py`.

`Element` handles:

- element identity and connectivity (`i`, `j`)
- material and section assignment
- element length and orientation calculations
- local/global axis generation
- transformation and stiffness abstractions

This base class is extended by frame and truss elements.

### Material and section

Defined in:

- `src/core/materials/base_material.py`
- `src/core/sections/base_section.py`

`Material` stores elastic properties like `E`, `G`, `nu`, and `gamma`.

`Section` stores section properties like `area`, `Ixx`, `Iyy`, and `J`.

### Frame element

Defined in `src/core/elements/frame.py`.

A `Frame` is a 3D 6-DOF-per-node member model with:

- axial deformation
- torsion
- bending in both local axes
- element release support via `release()` and release-aware stiffness reduction

The local stiffness matrix is assembled using standard Euler-Bernoulli style frame stiffness terms, and the transformation matrix is generated from the member local axes.

### Beam element

Defined in `src/core/elements/beam.py`.

`Beam` inherits from `Frame` and removes the axial and torsional DOFs to create a planar beam formulation. It is intended for bending-dominated beam behavior and uses only the in-plane bending and shear DOFs relevant to beam analysis.

### Truss element

Defined in `src/core/elements/truss.py`.

A `Truss` is a 3D axial member with nodal translational DOFs only. It does not support DOF releases and computes a simple axial stiffness contribution.

## Loads and load combinations

### Nodal loads

Defined in `src/core/loads/nodal_load.py`.

A `NodalLoad` is a single point load acting on a specific node and DOF.

### Element loads

Defined in `src/core/loads/element_load.py`.

Supported element load types include:

- `UDL`: uniform distributed load
- `SelfWeight`: gravity-based distributed load
- `PointLoad`: point load at a distance from node i

These loads generate fixed-end force vectors and are converted into equivalent global nodal forces during the solver workflow.

### Load case and combinations

Defined in:

- `src/core/loads/load_case.py`
- `src/core/loads/load_combo.py`

A `LoadCase` groups several loads, and a `LoadCombination` maps each load case to a multiplier factor, such as:

```python
LC1 = LoadCombination(
    name="LC1",
    loadCaseAndFactors={DEAD_LOAD: 1.0}
)
```

## Preprocessing and solving

### Preprocess

Defined in `src/core/analysis/preprocessing.py`.

`Preprocess(model)` performs the following:

- validates the model
- checks for missing or invalid nodes/elements
- assigns model-level DOF indices to each node
- assembles the global stiffness matrix
- checks stability of the free-DOF stiffness matrix

### Linear static solve

Defined in `src/core/analysis/linear_static.py`.

The solver does the following:

- builds the full force vector from nodal and element loads
- extracts the free DOF system
- solves:

```python
K_ff * D_f = F_f
```

- stores displacement results in the model
- computes reactions as:

```python
R = K * D - F
```

- builds a `SolutionState` with result data for nodes and elements

## Result model

Defined in `src/core/results/solution_state.py`.

The `SolutionState` object provides convenience methods such as:

- `node_displacement(node_id, dof)`
- `node_reaction(node_id, dof)`
- `local_element_end_force(element_id, node, dof)`
- `internal_force_axial(...)`
- `internal_force_shear_y(...)`
- `internal_force_moment_z(...)`

This makes it easy to query the final structural response after analysis.

## Visualization

Defined in `src/visualization/viewer.py`.

The `SolutionStateViewer` class uses `pyvista` to draw:

- original geometry
- deformed geometry
- nodes and elements
- restraint symbols
- axes and bounds

The viewer is created from a solved `SolutionState` and may be used as:

```python
viewer = SolutionStateViewer(solution, deformation_scale=50.0)
viewer.show()
```

## Example usage

The examples under `examples_new/` are the best starting point. The fixed beam example demonstrates the basic workflow:

```python
from src.core.geometry.node import Node
from src.core.elements.frame import Frame
from src.core.materials.base_material import Material
from src.core.sections.base_section import Section
from src.core.model import Model
from src.core.loads.load_case import LoadCase
from src.core.loads.load_combo import LoadCombination
from src.core.loads.nodal_load import NodalLoad
from src.core.analysis.preprocessing import Preprocess
from src.core.analysis.linear_static import LinearStaticSolve
from src.utils import global_variables as gv

N1 = Node("N1", 0.0, 0.0, 0.0)
N2 = Node("N2", 5000.0, 0.0, 0.0)
N3 = Node("N3", 10000.0, 0.0, 0.0)

for node in (N1, N3):
    for dof in gv.GLOBAL_DISP_DOFS:
        node.restrain(dof)

steel = Material(material_id="A36_STEEL", E=200000, nu=0.3)
section = Section(section_id="W200x15", area=1910, Ixx=12.8e6, Iyy=0.87e6, J=17.7e3)

E1 = Frame("E1", N1, N2, steel, section)
E2 = Frame("E2", N2, N3, steel, section)

model = Model()
model.add_node(N1)
model.add_node(N2)
model.add_node(N3)
model.add_element(E1)
model.add_element(E2)

N2_UY = NodalLoad(id="N2_UY", node=N2, dof=gv.UY, magnitude=-1000.0)
load_case = LoadCase(name="Dead_Load")
load_case.add_nodal_load(N2_UY)
load_combo = LoadCombination(name="LC1", loadCaseAndFactors={load_case: 1.0})

Preprocess(model)
solution = LinearStaticSolve(model, load_combo)

print(solution.node_displacement(N2.id, gv.UY))
```

## Example scripts

The following examples are included in the repository:

- `examples_new/fixed_ended_beam.py`
- `examples_new/cantilever.py`
- `examples_new/propped_cantilever.py`
- `examples_new/portal_frame.py`
- `examples_new/slanted_cantilever.py`
- `examples_new/space_frame.py`
- `examples_new/space_truss.py`

These scripts show how to build models, apply loads, solve, and view the structural response.

## Global DOF convention

The project uses a 6-DOF per node convention for 3D frame behavior:

- `UX`, `UY`, `UZ` = translations
- `RX`, `RY`, `RZ` = rotations

The constants are defined in `src/utils/global_variables.py`.

## Important implementation notes

This project is best understood as an experimental structural analysis framework. A few implementation details are worth noting:

- the main active code path is under `src/`, while `source/` appears to be an older or partial legacy structure
- some methods, comments, and variable names suggest an evolving prototype rather than a finalized API
- there are deprecated internal stress-accessor sections in `Element` that are not central to the current solver flow
- the examples use direct imports from `src` and run in a Python environment with the package on the `PYTHONPATH`
- `SolutionStateViewer` relies on PyVista and is intended for interactive structural visualization

## Recommended usage pattern

For new work, the recommended pattern is:

```python
model = Model()
# add nodes, elements, materials, sections
Preprocess(model)
solution = LinearStaticSolve(model, load_combo)
```

Then inspect the results through `solution.node_displacement()`, `solution.node_reaction()`, or `solution.internal_force_*()` helpers.

## Future development directions

The codebase is a good base for adding:

- more element types (shell, plate, membrane, spring)
- more load types and combinations
- modal / buckling / nonlinear analysis
- richer result post-processing
- better validation and unit tests
- a cleaner public API and packaging layout

## Summary

OSSTRAND is a compact Python structural analysis library focused on linear static analysis of truss and frame systems. It is simple to follow, well-suited for teaching and prototyping, and already demonstrates the full analysis process from model creation to visualization.

If you are learning structural analysis or prototyping a solver workflow, this repository is a strong starting point.

