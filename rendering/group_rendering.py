from rendering.object_rendering import render_obstacle, render_pipe
import vpython as vpy
import numpy as np
from path_finding.p_math import diff_nodes
from path_finding.restriction_functions import get_direction_of_pos
from data_class.Solution import Solution
from typing import Optional

"""functions for rendering a group of objects"""

def render_obstacles_from_state_grid(state_grid, rendering_grid, scene) -> Optional[vpy.compound]:
    """initialises obstacles from a preset to a scene (therefore rendering them) and grouping them as a compound"""
    obstacle_compound_list = []
    for index, state in np.ndenumerate(state_grid):
        rendering_grid_pos = vpy.vector(rendering_grid[index])
        if state == 1:  # spot is obstructed
            obstacle = render_obstacle(scene, rendering_grid_pos)
            obstacle_compound_list.append(obstacle)
    if not obstacle_compound_list:
        return None
    obstacle_compound = vpy.compound(obstacle_compound_list)
    return obstacle_compound


# todo: get directions from solution data class
def render_solution(parts_used:list, path:list, rendering_grid: np.ndarray, scene: vpy.canvas) -> list[vpy.cylinder]:
    """fully renders a given solution (regardless of obstruction) + start/goal points"""
    solution_layout = []
    for index, part_id in enumerate(parts_used):
        #skip the first entry
        if index == 0 or part_id== 0:
            continue
        pipe = render_pipe(scene=scene, pos=rendering_grid[path[index-1]],
                    direction=get_direction_of_pos(diff_nodes(path[index-1], path[index])), pipe_data=None,
                    part_id=part_id)
        solution_layout.append(pipe)

    return solution_layout









