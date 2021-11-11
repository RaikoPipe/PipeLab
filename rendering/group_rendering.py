from object_rendering import render_obstacle
import vpython as vpy
import numpy as np

"""functions for rendering a group of objects"""

def render_obstacles_from_preset(state_grid, coord_grid, scene) -> vpy.compound:
    """initialises obstacles from a preset to a scene (therefore rendering them) and grouping them as a compound"""
    obstacle_compound_list = []
    for index, state in np.ndenumerate(state_grid):
        coord_grid_pos = coord_grid[index]
        if state == 1:  # spot is obstructed
            obstacle = render_obstacle(coord_grid_pos, scene)
            obstacle_compound_list.append(obstacle)
    obstacle_compound = vpy.compound(obstacle_compound_list)
    return obstacle_compound


# todo: To be finished
def render_solution(solution, state_grid, coord_grid, scene) -> vpy.compound:
    """fully renders a given solution (regardless of obstruction) + start/goal points"""
    for index, state in np.ndenumerate(state_grid):
        coord_grid_pos = coord_grid[index]
        if state == 1:  # spot is obstructed
            render_obstacle(coord_grid_pos, scene)

