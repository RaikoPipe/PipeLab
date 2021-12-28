from path_finding.search_algorithm import find_path
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from grid.grid_functions import get_empty_stategrid, change_grid_states
import numpy as np

#todo: use change_grid_states function to create a mounting wall with obstacles
#todo: create a solution with a* and extract path and parts to define a static solution

from rendering import object_rendering, group_rendering
from grid import grid_functions, randomizer

x=20
y=20

r_grid, mounting_wall_data = grid_functions.get_rendering_grid(x,y)
solution = None
while solution is None:
    state_grid = grid_functions.get_empty_stategrid(x, y)
    state_grid = randomizer.set_random_obstacles(0.1, state_grid)
    start_node = (0,0)
    goal_node = (17,19)
    state_grid[start_node] = 0
    state_grid[goal_node] = 0

    pipe_stock = {0: 100, 1: 100, 2: 100, 3: 100, 4: 100, 5: 100, 6: 100}

    part_cost = {0: 5.32, 1: 3.00, 2: 3.00, 3: 3.00, 4: 3.00, 5: 3.00, 6: 00}

    weights = Weights(path_length=1, cost=0, distance_to_obstacles=0)

    path_problem = PathProblem(state_grid=state_grid, start_node=start_node, goal_node=goal_node, start_direction=(1, 0),
                               goal_direction=(0, 1), goal_is_transition=False, part_cost=part_cost,
                               starting_part=None, part_stock=pipe_stock, weights=weights, algorithm="mcsa*")



    solution = find_path(path_problem=path_problem)

print(path_problem)
print(solution)

#rendering

scene = object_rendering.create_new_scene()
mounting_wall, dot_object = \
    object_rendering.render_mounting_wall(scene=scene, rendering_grid=r_grid, mounting_wall_data=mounting_wall_data)

obstacles = group_rendering.render_obstacles_from_state_grid(state_grid=state_grid, rendering_grid=r_grid, scene=scene)

solution_layout = group_rendering.render_pipe_layout(solution.parts, solution.path, r_grid, scene)

from EventHandler import get_trails_from_state_grid

state_grid = np.array([[0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 1, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 1, 0, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0, 0, 0],
       [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
       [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 2],
       [0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 2],
       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0]])

print(get_trails_from_state_grid(state_grid))
