# Nodes
NODE_i, NODE_j = 0, 1

# Global Force DOFs
FX, FY, FZ = 0, 1, 2
MX, MY, MZ = 3, 4, 5
GLOBAL_FORCES = (FX, FY, FZ, MX, MY, MZ)
GLOBAL_FORCES_TRUSS = (FX, FY, FZ)

# Local Force DOFs
Nx, Vy, Vz = 0, 1, 2
Tx, My, Mz = 3, 4, 5
LOCAL_FORCES_FRAME = (Nx, Vy, Vz, Tx, My, Mz)
LOCAL_FORCES_BEAM  = (Vy, Vz, My, Mz)
LOCAL_FORCES_TRUSS = (Nx)

# Global Displacement DOFs
UX, UY, UZ = 0, 1, 2
RX, RY, RZ = 3, 4, 5
GLOBAL_DISP_DOFS = (UX, UY, UZ, RX, RY, RZ)
GLOBAL_DISP_DOFS_TRUSS = (UX, UY, UZ)

# Local Displacement DOFs
ux, uy, uz = 0, 1, 2
rx, ry, rz = 3, 4, 5
LOCAL_DISP_DOFS_FRAME = (ux, uy, uz, rx, ry, rz)
LOCAL_DISP_DOFS_BEAM  = (uy, uz, ry, rz)
LOCAL_DISP_DOFS_TRUSS = (ux)