#sandbox/test_nodes.py

from source.model.nodes import Node

# Create some nodes
n1 = Node(1, 0.0, 0.0)
n2 = Node(2, 3.0, 0.0)
n3 = Node(3, 3.0, 4.0)

def distance(a: Node, b: Node) -> float:
    dx = b.x - a.x
    dy = b.y - a.y
    dz = b.z - a.z
    return (dx**2 + dy**2 + dz**2) ** 0.5

print(distance(n1, n3))

