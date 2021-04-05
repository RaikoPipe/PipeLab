from vpython import *

size = 1
sizeVector = vector(size * 5, size * 5, 5)
c1 = cylinder(size=sizeVector, axis=vector(0, 1, 0),
                   color=color.cyan)
c2 = cylinder(size=sizeVector, axis=vector(1, 0, 0),
                   color=color.cyan)
c3 = sphere(size=sizeVector, axis=vector(1, 0, 0),
                   color=color.cyan)
corner = compound([c1, c2])