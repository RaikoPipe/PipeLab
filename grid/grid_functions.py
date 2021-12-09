import numpy as np
import vpython as vpy
from rendering.object_data_classes import mounting_wall_data

"""contains grid class and all functions concerning manipulating its contents"""

class GridPreset:
    """class that contains the state_grid, coord_grid and solutions to it"""
    def __init__(self):
        self.state_grid = None
        self.coord_grid = None
        self.solutions = []

    def init_empty_grid(self, x_length: int, y_length: int):
        self.state_grid = get_empty_stategrid(x_length, y_length)
        self.coord_grid = get_rendering_grid(x_length, y_length)


def get_empty_stategrid(x_length: int, y_length: int):
    """returns state grid according to input parameters"""
    state_grid = np.tile(0, (x_length, y_length))
    return state_grid

def change_grid_states(state_grid: np.ndarray, node_states: list): # [((0,0),1),((0,1),1)]
    """modify grid states according to items in position_states"""

    for item in node_states:
        state_grid[item[0]] = item[1] # 0: node, 1: state

    return state_grid

# defaults
x_start_default = 50
y_start_default = 100
z_start_default = 150
dot_dist_default = 105

def get_rendering_grid(x_length:int, y_length:int, x_start:float = x_start_default,
                       y_start:float = y_start_default, z_start:float = z_start_default, dot_dist:float = dot_dist_default) -> \
tuple[np.ndarray, mounting_wall_data]:
    """create a grid that contains 2D-coordinates for each position"""
    rendering_grid = np.zeros((x_length, y_length), dtype=vpy.vector)

    x_pos = x_start
    y_pos = y_start
    z_pos = z_start

    for x in range(0, x_length):
        y = 0
        y_pos = y_start
        rendering_grid[x, y] = vpy.vector(x_pos, y_pos, z_pos)

        # create dots along y axis
        for y in range(0, y_length):
            y_pos += dot_dist

            rendering_grid[x, y] = vpy.vector(x_pos, y_pos, z_pos)

        x_pos += dot_dist

    size_data = mounting_wall_data(dim= vpy.vector(x_pos + x_start - dot_dist, 2*y_start + y_pos, z_pos))
    return rendering_grid, size_data

