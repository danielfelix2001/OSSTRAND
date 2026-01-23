# src/model/loads/load_combo.py

class LoadCombination:
    """
    factors: dict {load_case_name: factor}
    example: {DL:1.2, LL:1.6}
    """
    def __init__(self, name:str, loadCaseAndFactors):
        self.name = name
        self.loadCaseAndFactors = loadCaseAndFactors