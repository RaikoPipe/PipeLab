from copy import deepcopy

import numpy as np

from data_class.PathProblem import PathProblem
from path_finding.path_utilities import get_outgoing_pos, get_best_connections, get_connections
import heapq
from path_finding.search_algorithm import find_path
from path_finding.path_math import diff_pos, get_direction
from path_finding.common_types import Layouts

def find_solution_from_partial_construction(state_grid:np.ndarray, layout_directions: dict, outgoing_connections_set: set, initial_path_problem):
    exclusion_list = set()

    tentative_path_problem = deepcopy(initial_path_problem)
    tentative_path_problem.state_grid = state_grid




    while True:
        connections = get_connections(exclusion_list=exclusion_list, outgoing_connections_set=outgoing_connections_set)
        for connection in connections:
            tentative_path_problem.start_pos = connection[0]
            tentative_path_problem.goal_node = connection[1]
            #fixme: also add start_direction, goal_directions depending on trail direction
            solution = find_path(tentative_path_problem)
            if solution is None:
                # no connection found, add connection to exclusion list, start over
                exclusion_list.add(connection)
                break

            partial_solutions.append(solution)

        if connections == exclusion_list:
            return None

        return partial_solutions

def find_partial_solution_ls(layouts: Layouts, state_grid: np.ndarray,
                                 initial_path_problem: PathProblem, completed_set:set):


    tentative_path_problem = deepcopy(initial_path_problem)
    tentative_path_problem.state_grid = state_grid
    partial_solutions = []


    node_dict = {}
    layout_id = 0

    for connection in completed_set:
        # add fit_pos, get direction of the layout (horizontal/vertical)
        layout_dir = get_direction(diff_pos(connection[0], connection[1]))
        node_dict[(connection[0],layout_id)] = layout_dir
        node_dict[(connection[1], layout_id)] = layout_dir
        layout_id += 1

    exclusion_list = set() # for keeping track of connections with no valid path

    while True:
        connections = get_best_connections(node_dict=node_dict, exclusion_list=exclusion_list)
        for connection in connections:
            tentative_path_problem.start_pos = connection[0]
            tentative_path_problem.goal_node = connection[1]
            #fixme: also add start_direction, goal_directions depending on trail direction
            solution = find_path(tentative_path_problem)
            if solution is None:
                # no connection found, add connection to exclusion list, start over
                exclusion_list.add(connection)
                break

            partial_solutions.append(solution)

        if connections == exclusion_list:
            return None

        return partial_solutions

def find_partial_solution_simple(layout_paths: list, captured_state_grid: np.ndarray,
                                 initial_path_problem: PathProblem):
    tentative_path_problem = deepcopy(initial_path_problem)
    tentative_path_problem.state_grid = captured_state_grid
    partial_solutions = []

    node_set = get_outgoing_pos(paths=layout_paths, first_pos=tentative_path_problem.start_pos,
                                last_pos=tentative_path_problem.goal_node)

    exclusion_list = set()

    while True:
        connections = get_best_connections(node_dict=node_set, exclusion_list=exclusion_list)
        for connection in connections:
            tentative_path_problem.start_pos = connection[0]
            tentative_path_problem.goal_node = connection[1]
            solution = find_path(tentative_path_problem)
            if solution is None:
                # no connection found, add connection to exclusion list, start over
                exclusion_list.add(connection)
                break

            partial_solutions.append(solution)

        if connections == exclusion_list:
            return None

        return partial_solutions

# thoughts:

    # todo: - put random parts on grid
    #       - identify parts on grid (with path?)
    #       - determine goal/start point of each part
    #       - try to create solution for each possible combination
    #       - solution with lowest score wins

    # todo: need function to detect if worker follows optimal solution

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

    # todo: Implement easier way to determine which outgoing layout nodes to connect to each other

    # start_best_solution = get_best_solution_from_nodes(tentative_path_problem, connecting_nodes)
    # connecting_nodes.discard(start_best_solution.path_problem.goal_node)
    # partial_solutions["start"] = start_best_solution
    #
    # tentative_path_problem.goal_node = initial_path_problem.start_node
    # #todo: reverse path, parts used in following solution
    # goal_best_solution = get_best_solution_from_nodes(tentative_path_problem, connecting_nodes)
    # connecting_nodes.discard(goal_best_solution.path_problem.goal_node)
    # partial_solutions["goal"] = goal_best_solution

    # todo: can we do something with all the solutions we get connecting to start/goal?
    # todo: instead of finding solutions to connecting each start/goal/layout to each other, simply determine all outgoing nodes (two for each layout),
    #   then try to connect them with solution paths-> much more efficient, but lower solution quality

# def find_partial_solution_complex(layout_paths: list, captured_state_grid: np.ndarray, initial_path_problem: PathProblem):
#
#
#     # check which part should be the firsts
#     solutions_to_first_part = []
#     solutions_to_goal = []
#     # each path can be a layout of one or more parts, that can be connected to start xor goal
#
#     # todo: easy implementationidea:
#     #  just evaluate each first/last node of a layout and score determines order in which layout connects
#     #   disadvantage: cant evaluate paths by cost/mino anymore
#
#     tentative_path_problem = deepcopy(initial_path_problem)
#     tentative_path_problem.state_grid = captured_state_grid
#     connecting_nodes = get_outgoing_nodes(layout_paths=layout_paths)
#     partial_solutions = {}
#
#     start_best_solution = get_best_solution_from_nodes(tentative_path_problem, connecting_nodes)
#     connecting_nodes.discard(start_best_solution.path_problem.goal_node)
#     partial_solutions["start"] = start_best_solution
#
#     tentative_path_problem.goal_node = initial_path_problem.start_node
#     #todo: reverse path, parts used in following solution
#     goal_best_solution = get_best_solution_from_nodes(tentative_path_problem, connecting_nodes)
#     connecting_nodes.discard(goal_best_solution.path_problem.goal_node)
#     partial_solutions["goal"] = goal_best_solution
#
#     # todo: can we do something with all the solutions we get connecting to start/goal?
#
#
#
#
#     for layout_path in layout_paths:
#         """identify layout that connects to start"""
#         first_node = layout_path[0]
#         last_node = layout_path[-1]
#
#         # first try to get solution from start to first node
#         tentative_path_problem.goal_node = first_node
#         solution_first = find_path(tentative_path_problem)
#
#         # then try to get solution from start to last node
#         tentative_path_problem.goal_node = last_node
#         solution_last = find_path(tentative_path_problem)
#         #compare the two solutions
#         if solution_first.score >= solution_last.score:
#             solutions_to_first_part.append(solution_last)
#
#         else:
#             solutions_to_first_part.append(solution_first)
#
#         # todo somehow get axis
#         # todo: update part_stock
#
#
#     if len(layout_paths) == 1:
