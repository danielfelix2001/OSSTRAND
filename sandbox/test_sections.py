#sandbox/test_sections.py

import sys
sys.path.append(".")
from src.model.sections import Section, SteelSection

W_section = SteelSection(
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
print(W_section)