from object_rendering import render_obstacle
import numpy as np

"""functions for initialising grid presets"""

def render_preset(state_grid, coord_grid, scene):
    """render a given preset"""
    for index, state in np.ndenumerate(state_grid):
        coord_grid_pos = coord_grid[index]
        if state == 1: # spot is obstructed
            render_obstacle(coord_grid_pos, scene)



from path_finding.state_grid_functions import get_empty_stategrid,change_grid_states
from coord_grid_functions import get_coord_grid
from vpython import canvas
# debug code
positionStates = [{"pos": (1, 2), "state": 1}, {"pos": (3, 3), "state": 1}]
g = get_empty_stategrid(10, 10)
g = change_grid_states(g, positionStates)

c_grid = get_coord_grid(g.shape[0], g.shape[1])
render_preset(g, c_grid, scene=canvas)

print(g)