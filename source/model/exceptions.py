# source/model/exceptions.py

class MSAError(Exception):
    """Base class for all MSA-related errors."""
    pass

class ModelDefinitionError(MSAError):
    """Errors in model setup (nodes, elements, loads)."""
    pass

class ElementError(MSAError):
    """Invalid element definitions."""
    pass

class StabilityError(MSAError):
    """Structure is unstable or ill-conditioned."""
    pass

class DOFError(MSAError):
    pass

class SingularMatrixError(MSAError):
    """Global stiffness matrix is singular."""
    pass

class SolverError(MSAError):
    """Generic solver failure."""
    pass