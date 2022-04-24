from typing import Tuple

import numpy as np
import vpython as vpy

from type_dictionary.common_types import *
from rendering.object_data_classes import mounting_wall_data


def get_empty_stategrid(x_nodes: int, y_nodes: int):
    """Returns state grid according to parameters
    Args:
        x_nodes: number of nodes on the x-axis
        y_nodes: number of nodes on the y-axis
        """
    state_grid = np.tile(0, (x_nodes, y_nodes))
    return state_grid


def change_grid_states(state_grid: np.ndarray, node_states: list[Node]):
    """Modify grid states according to items in node_states
    Args:
        node_states: list containing Nodes
        state_grid:

    """

    for item in node_states:
        if state_grid[item[0]] == 3:
            # transition states can't be overwritten
            continue
        state_grid[item[0]] = item[1]  # 0: pos, 1: state

    return state_grid


# defaults
x_start_default = 50
y_start_default = 100
z_start_default = 150
dot_dist_default = 105

def get_rendering_grid(x_nodes: int, y_nodes: int, x_start: float = x_start_default,
                       y_start: float = y_start_default, z_start: float = z_start_default,
                       node_dist: float = dot_dist_default) -> \
        Tuple[np.ndarray, mounting_wall_data]:
    """create a grid that contains 2D-coordinates for each position
    Args:
        x_nodes: number of nodes on the x-axis
        y_nodes: number of nodes on the y-axis
        x_start: x coordinate value that signifies position of the first node on the x axis
        y_start: y coordinate value that signifies position of the first node on the y axis
        z_start: z coordinate value of spacing of nodes to the x-y-plane
        node_dist: distance between each node on the x-y-plane
    """
    rendering_grid = np.zeros((x_nodes, y_nodes), dtype=vpy.vector)

    x_pos = x_start
    y_pos = y_start
    z_pos = z_start

    for x in range(0, x_nodes):
        y = 0
        y_pos = y_start
        rendering_grid[x, y] = vpy.vector(x_pos, y_pos, z_pos)

        # create dots along y axis
        for y in range(0, y_nodes):
            y_pos += node_dist

            rendering_grid[x, y] = vpy.vector(x_pos, y_pos, z_pos)

        x_pos += node_dist

    size_data = mounting_wall_data(dim=vpy.vector(x_pos + x_start - node_dist, 2 * y_start + y_pos, z_pos))
    return rendering_grid, size_data


def set_transition_points(state_grid, transition_points_set):
    for pos, state in np.ndenumerate(state_grid):
        for transition_point in transition_points_set:
            if pos[0] == transition_point[0] or pos[1] == transition_point[1]:
                state_grid[pos] = 3

    return state_grid