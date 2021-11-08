import numpy as np

"""contains functions for creating and modifying the state_grid"""

def change_grid_states(state_grid, position_states: list):
    """modify grid states according to items in position_states"""

    for item in position_states:
        state_grid[item["pos"]] = item["state"]

    return state_grid

def get_empty_stategrid(x_length: int, y_length: int):
    """returns state grid according to input parameters"""
    state_grid = np.tile(0, (x_length, y_length))
    return state_grid
