class Section:
    def __init__(self, section_id: str, 
                 area: float,
                 Ixx: float = 0.0, Iyy: float = 0.0, J: float = 0.0):
        self.id = section_id
        self.area = area
        self.Ixx = Ixx  # about strong axis
        self.Iyy = Iyy
        self.J = J