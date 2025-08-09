import os
import sys
import numpy as np

vertices = []
triangles = []

# load the obj file and modify all the y-coordinated to 100 higher
with open(os.path.join(os.path.dirname(__file__), 'cloth.obj'), 'r') as f:
    lines = f.readlines()
    for line in lines:
        if line.startswith('v '):
            parts = line.split()
            x, y, z = map(float, parts[1:4])
            vertices.append((x, -(y + 100), z))  # Increase y-coordinate by 100
        elif line.startswith('f '):
            parts = line.split()
            # Convert face indices to zero-based and store as integers
            face = [int(part.split('/')[0]) - 1 for part in parts[1:]]
            triangles.append(face)

# Write the modified vertices and triangles back to a new obj file
output_file = os.path.join(os.path.dirname(__file__), 'modified_cloth.obj')
with open(output_file, 'w') as f:
    for v in vertices:
        f.write(f"v {v[0]} {v[1]} {v[2]}\n")
    for t in triangles:
        f.write(f"f {' '.join(str(i + 1) for i in t)}\n")  # Convert back to one-based indexing