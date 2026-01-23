# src/utils/global_variables.py

# Global Force DOFs
FX, FY, FZ = 0, 1, 2
MX, MY, MZ = 3, 4, 5
GLOBAL_FORCES = (FX, FY, FZ, MX, MY, MZ)

# Local Force DOFs
Nx, Vy, Vz = 0, 1, 2
Tx, My, Mz = 3, 4, 5
LOCAL_FORCES = (Nx, Vy, Vz, Tx, My, Mz)

# Global Displacement DOFs
UX, UY, UZ = 0, 1, 2
RX, RY, RZ = 3, 4, 5
GLOBAL_DISP_DOFS = (UX, UY, UZ, RX, RY, RZ)

# Local Displacement DOFs
ux, uy, uz = 0, 1, 2
rx, ry, rz = 3, 4, 5
LOCAL_DISP_DOFS = (ux, uy, uz, rx, ry, rz)