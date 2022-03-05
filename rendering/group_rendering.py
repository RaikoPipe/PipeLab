from rendering.object_rendering import render_obstacle, render_pipe, render_corner
import vpython as vpy
import numpy as np
from path_finding.p_math import diff_pos, sum_pos
from path_finding.path_utilities import get_diagonal_direction, get_direction
from data_class.Solution import Solution
from typing import Optional

"""functions for rendering a group of objects"""

def render_obstacles_from_state_grid(state_grid: np.ndarray, rendering_grid:np.ndarray, scene:vpy.canvas) -> Optional[vpy.compound]:
    """Renders obstacles according to obstructions on the state_grid to VPython canvas.
    Returns a list containing the rendered objects, if any were created."""
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
def render_pipe_layout(parts_used:list, path:list, rendering_grid: np.ndarray, scene: vpy.canvas) -> list[vpy.cylinder]:
    """Renders pipe objects on a VPython canvas according to the given list of parts used."""
    solution_layout = []
    for index, part_id in enumerate(parts_used):
        # skip the first entry
        if index == 0:
            continue
        elif part_id == 0:
            # create corner
            from_dir = get_direction(diff_pos(path[index - 1], path[index]))
            to_dir = get_direction(diff_pos(path[index], path[index + 1]))

            pos = rendering_grid[path[index]]
            corner = render_corner(scene=scene, pos=pos, course=(from_dir, to_dir))
            solution_layout.append(corner)

        else:
            direction = get_direction(diff_pos(path[index - 1], path[index]))
            pos = rendering_grid[path[index-1]]
            pipe = render_pipe(scene=scene, pos=pos,
                        direction=direction, pipe_data=None,
                        part_id=part_id)
            solution_layout.append(pipe)

    return solution_layout









