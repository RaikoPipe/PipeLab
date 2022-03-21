from rendering.object_rendering import render_obstacle, render_pipe, render_corner, render_marker
import vpython as vpy
import numpy as np
from path_finding.path_math import diff_pos, sum_pos, get_diagonal_direction, get_direction
from data_class.Solution import Solution
from typing import Optional
from path_finding.common_types import *
from rendering import object_data_classes as odc
from rendering.default_data import *

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
def render_pipe_layout(definite_path: DefinitePath, rendering_grid: np.ndarray, scene: vpy.canvas, opacity:float) -> list[vpy.cylinder]:
    """Renders pipe objects on a VPython canvas according to the given list of parts used."""
    solution_layout = []
    for index, (pos, part_id) in enumerate(definite_path):
        # skip the first entry
        if index == 0:
            continue
        elif part_id == 0:
            # create corner
            from_dir = get_direction(diff_pos(definite_path[index - 1][0], definite_path[index][0]))
            try:
                to_dir = get_direction(diff_pos(definite_path[index][0], definite_path[index + 1][0]))
            except IndexError:
                print("Vpython rendering: No next object in list, finished rendering.")
                break


            pos = rendering_grid[definite_path[index][0]]
            corner = render_corner(scene=scene, pos=pos, course=(from_dir, to_dir), opacity=opacity)
            solution_layout.append(corner)

        else:
            direction = get_direction(diff_pos(definite_path[index - 1][0], definite_path[index][0]))
            pos = rendering_grid[definite_path[index-1][0]]
            pipe = render_pipe(scene=scene, pos=pos,
                        direction=direction,
                        part_id=part_id, opacity=opacity)
            solution_layout.append(pipe)

    return solution_layout

def render_start_goal_markers(scene:vpy.canvas, start_pos:Pos, goal_pos:Pos, rendering_grid: np.ndarray):
    start_coord = rendering_grid[start_pos]
    goal_coord = rendering_grid[goal_pos]
    start_marker = render_marker(scene=scene, pos=start_coord,marker_color=vpy.color.green)
    goal_marker = render_marker(scene=scene, pos=goal_coord, marker_color=vpy.color.red)
    return start_marker, goal_marker









