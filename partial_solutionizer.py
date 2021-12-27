from path_finding.search_algorithm import find_path
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from data_class.Solution import Solution
from grid.grid_functions import get_empty_stategrid, change_grid_states
from copy import deepcopy
import numpy as np
from typing import Optional





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

def get_solutions_from_nodes(path_problem, connecting_nodes):
    # get scores for connecting to start
    solutions = []
    for connecting_node in connecting_nodes:
        path_problem.goal_node = connecting_node
        solution = find_path(path_problem)
        solutions.append(solution)
        # todo: get axis of connecting node somehow


    # todo: get solution with lowest score; use heapq
    best_solution = Solution() #todo: see line 71

    return best_solution

def something(layout_paths: list, captured_state_grid: np.ndarray, initial_path_problem: PathProblem):
    # here we try to help with whatever the worker is trying to build if he deviates from optimal solution
    # todo: how do we know the worker deviated from the path?
    #  1. still on optimal path, but different part used
    #  2. part he placed is outside optimal path
    #  -> function for identifying deviation is needed (get path from state_grid)
    #   in both cases he deviated completely and we need to figure out what he is trying to do
    #  try to connect each part by searching for the shortest path from node to node
    #  If goal cant be reached from current part, try to reach goal from each previous part
    #  might be useful: checking if goal can be reached from any part (what information can we get with this?)
    #  possible use cases: identifying which part should be last
    #  idea: identify first part, then last, then try to connect parts in between.

    # check which part should be the firsts
    solutions_to_first_part = []
    solutions_to_goal = []
    # each path can be a layout of one or more parts, that can be connected to start xor goal

    # todo: easy implementationidea:
    #  just evaluate each first/last node of a layout and score determines order in which layout connects
    #   disadvantage: cant evaluate paths by cost/mino anymore

    tentative_path_problem = deepcopy(initial_path_problem)
    tentative_path_problem.state_grid = captured_state_grid
    connecting_nodes = set()
    partial_solutions = {}

    #add each first/last node into a list
    for layout_path in layout_paths:
        connecting_nodes.add(layout_path[0])
        connecting_nodes.add(layout_path[-1])


    start_best_solution = get_solutions_from_nodes(tentative_path_problem, connecting_nodes)
    connecting_nodes.discard(start_best_solution.path_problem.goal_node)
    partial_solutions["start"] = start_best_solution

    tentative_path_problem.goal_node = path_problem.start_node
    #todo: reverse path, parts used in following solution
    goal_best_solution = get_solutions_from_nodes(tentative_path_problem, connecting_nodes)
    connecting_nodes.discard(goal_best_solution.path_problem.goal_node)
    partial_solutions["goal"] = goal_best_solution

    # todo: can we do something with all the solutions we get connecting to start/goal?




    for layout_path in layout_paths:
        """identify layout that connects to start"""
        first_node = layout_path[0]
        last_node = layout_path[-1]

        # first try to get solution from start to first node
        tentative_path_problem.goal_node = first_node
        solution_first = find_path(tentative_path_problem)

        # then try to get solution from start to last node
        tentative_path_problem.goal_node = last_node
        solution_last = find_path(tentative_path_problem)
        #compare the two solutions
        if solution_first.score >= solution_last.score:
            solutions_to_first_part.append(solution_last)

        else:
            solutions_to_first_part.append(solution_first)

        # todo somehow get axis
        # todo: update part_stock


    if len(layout_paths) == 1:

