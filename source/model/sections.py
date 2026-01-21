# source/model/sections.py

class Section:
    def __init__(self, section_id: str, 
                 area: float,
                 Ixx: float = 0.0, Iyy: float = 0.0, J: float = 0.0):
        self.id = section_id
        self.area = area
        self.Ixx = Ixx  # about strong axis
        self.Iyy = Iyy
        self.J = J
        #self.material = material

    def __repr__(self):
        return f"Section(id={self.id}, area={self.area}, Ixx={self.Ixx}, Iyy={self.Iyy}, J={self.J})"
    
class Section(Section):
    def __init__(self, section_id: str, 
                 area: float,
                 Ixx: float = 0.0, Iyy: float = 0.0, J: float = 0.0,                 
                 shape_type: str = "",
                 Sx: float = 0.0, Sy: float = 0.0,
                 Zx: float = 0.0, Zy: float = 0.0):
        super().__init__(section_id, area, Ixx, Iyy, J)
        self.shape_type = shape_type
        self.Sx = Sx    #section modulus strong axis
        self.Sy = Sy
        self.Zx = Zx    #plastic section modulus strong axis
        self.Zy = Zy    
    
    def __repr__(self):
            return (f"SteelSection(id={self.id}, shape_type={self.shape_type}, "
                    f"area={self.area}, Ixx={self.Ixx}, Iyy={self.Iyy}, J={self.J}, "
                    f"Sx={self.Sx}, Sy={self.Sy}, Zx={self.Zx}, Zy={self.Zy})")
        
