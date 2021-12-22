from path_finding.search_algorithm import find_path
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from grid.grid_functions import get_empty_stategrid, change_grid_states
from copy import deepcopy
import numpy as np





from rendering import object_rendering, group_rendering
from grid import grid_functions, randomizer

x=20
y=20

r_grid, mounting_wall_data = grid_functions.get_rendering_grid(x,y)
pipe_stock = {0: 100, 1: 100, 2: 100, 3: 100, 4: 100, 5: 100, 6: 100}

part_cost = {0: 5.32, 1: 3.00, 2: 3.00, 3: 3.00, 4: 3.00, 5: 3.00, 6: 00}

solution = None
while solution is None:
    state_grid = grid_functions.get_empty_stategrid(x, y)
    state_grid = randomizer.set_random_obstacles(0.1, state_grid)

    weights = Weights(path_length=1, cost=0, distance_to_obstacles=0)

    path_problem = PathProblem(state_grid=state_grid, start_node=(0, 0), goal_node=(17, 19), start_direction=(1, 0),
                               goal_direction=(0, 1), goal_is_transition=False, part_cost=part_cost,
                               starting_part=None, part_stock=pipe_stock, weights=weights, algorithm="mcsa*")



    solution = find_path(path_problem=path_problem)

print(solution)

#rendering

scene = object_rendering.create_new_scene()
mounting_wall, dot_object = \
    object_rendering.render_mounting_wall(scene=scene, rendering_grid=r_grid, mounting_wall_data=mounting_wall_data)

obstacles = group_rendering.render_obstacles_from_state_grid(state_grid=state_grid, rendering_grid=r_grid, scene=scene)

solution_layout = group_rendering.render_pipe_layout(solution.parts, solution.path, r_grid, scene)


# todo: - put random parts on grid
#       - identify parts on grid (with path?)
#       - determine goal/start point of each part
#       - try to create solution for each possible combination
#       - solution with lowest score wins

# todo: need function to detect if worker follows optimal solution

def something(layout_paths: list, captured_state_grid: np.ndarray, initial_path_problem: PathProblem):
    # here we try to help with whatever the worker is trying to build if he deviates from optimal solution
    # todo: how do we know the worker deviated from the path?
    #  1. still on optimal path, but different part used
    #  2. part he placed is outside optimal path
    #   in both cases he deviated completely and we need to figure out what he is trying to do
    problems_to_solve = []
    partial_solutions = []
    something_path_problem = deepcopy(initial_path_problem)
    something_path_problem.state_grid = captured_state_grid
    # check which part should be the first
    solutions_to_first_part = []
    # each path can be a layout of one or more parts, that can be connected to start xor goal
    for layout_path in layout_paths:

        first_node = layout_path[0]
        last_node = layout_path[-1]

        # first try to get solution from start to first node
        something_path_problem.goal_node = first_node
        solution_first = find_path(something_path_problem)

        # then try to get solution from start to last node
        something_path_problem.goal_node = last_node
        solution_last = find_path(something_path_problem)

        #compare the two solutions
        if solution_first.score >= solution_last.score:
            solutions_to_first_part.append(solution_last)
        else:
            solutions_to_first_part.append(solution_first)

        # todo somehow get axis
        # todo: update part_stock

