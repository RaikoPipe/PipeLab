import numpy as np
import vpython as vpy

"""contains functions for positional operations of to-be-rendered objects; all measures are in m"""

#defaults
x_start_default = 50
y_start_default = 100
z_start_default = 150
dot_dist_default = 10.5

def get_coord_grid(x_length:int, y_length:int, x_start:float = x_start_default,
                   y_start:float = y_start_default, z_start:float = z_start_default, dot_dist:float = dot_dist_default) -> np.ndarray:
    """create a coordinate grid that contains 2D-coordinates for each position"""
    coord_grid = np.zeros((x_length, y_length), dtype=vpy.vector)

    x_pos = x_start
    y_pos = y_start
    z_pos = z_start

    for x in range(0, x_length):
        y = 0
        coord_grid[x, y] = vpy.vector(x_pos, y_pos, z_pos)

        # create dots along y axis
        for y in range(0, y_length):
            y_pos += dot_dist

            coord_grid[x, y] = vpy.vector(x_pos, y_pos, z_pos)

        x_pos += dot_dist
    return coord_grid

coord_matrix = get_coord_grid(1000, 1000, 50, 100, 10.5)
print(coord_matrix)

