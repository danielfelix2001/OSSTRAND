# src/utils/dof_helper.py

DOF_NAMES = {
    0: "UX", 1: "UY", 2: "UZ",
    3: "RX", 4: "RY", 5: "RZ"
}

GLOBAL_REACTION_NAMES = {
    0: "FX", 1: "FY", 2: "FZ",
    3: "MX", 4: "MY", 5: "MZ"
}

LOCAL_REACTION_NAMES = {
    0: "Nx", 1: "Vy", 2: "Vz",
    3: "Tx", 4: "My", 5: "Mz"
}

LOCAL_ELEMENT_REACTION_NAMES = {
    0: "Nx_i", 1: "Vy_i", 2: "Vz_i", 3: "Tx_i", 4: "My_i", 5: "Mz_i",
    6: "Nx_j", 7: "Vy_j", 8: "Vz_j", 9: "Tx_j", 10:"My_j", 11:"Mz_j"
}

def local_dof_map(element):
    """
    Converts DOF numbering to local fef vector indexing.\n
    For example, a Beam element with 8 DOFs:\n
        Node i = 0, Node j = 1
        DOF Numbering: 1 = uy, 2 = uz, 4 = ry, 5 = rz
        Local fef vector indexing:  0 = uy_i, 1 = uz_i, 2 = ry_i, 3 = rz_i, 
                                    4 = uy_j, 5 = uz_j, 6 = ry_j, 7 = rz_j
                                    
        Thus:   local_dof_map((0,4)) returns 2
                local_dof_map((1,5)) returns 7
    """
    dof_map = {}
    idx = 0
    for node in (0, 1):  # 0 = i, 1 = j
        for dof in element.NODE_DOF_INDICES:     # 0 = ux, 1 = uy, ..., 5 = rz
            dof_map[(node, dof)] = idx
            idx += 1
    return dof_map