#sandbox/test_elements.py

import sys
sys.path.append(".")

from src.model.nodes import Node
from src.model.lineElements.element import Element

n1 = Node(1, 0, 0)
n2 = Node(2, 3, 4)

e = Element(1, n1, n2)

print(e)
print("Length:", e.length())