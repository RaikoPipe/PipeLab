from copy import deepcopy

import numpy as np

from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
from data_class.BuildingInstruction import BuildingInstruction
from path_finding.path_utilities import get_outgoing_pos, get_best_connections, get_connections, construct_solution
import heapq
from path_finding.search_algorithm import find_path
from path_finding.path_math import diff_pos, get_direction, manhattan_distance
from path_finding.common_types import Layouts
from path_finding.solution_manager import get_solution
from path_finding.common_types import Trail
import constants as const


# def find_solution_from_partial_construction(state_grid:np.ndarray, layout_directions: dict, outgoing_connections_set: set, initial_path_problem):
#     exclusion_list = set()
#
#     tentative_path_problem = deepcopy(initial_path_problem)
#     tentative_path_problem.state_grid = state_grid
#
#
#
#
#     while True:
#         connections = get_connections(exclusion_list=exclusion_list, outgoing_connections_set=outgoing_connections_set)
#         for connection in connections:
#             tentative_path_problem.start_pos = connection[0]
#             tentative_path_problem.goal_node = connection[1]
#             #fixme: also add start_direction, goal_directions depending on trail direction
#             solution = find_path(tentative_path_problem)
#             if solution is None:
#                 # no connection found, add connection to exclusion list, start over
#                 exclusion_list.add(connection)
#                 break
#
#             partial_solutions.append(solution)
#
#         if connections == exclusion_list:
#             return None
#
#         return partial_solutions

def find_partial_solution_ls(layouts: Layouts, state_grid: np.ndarray,
                             initial_path_problem: PathProblem, completed_set: set):
    tentative_path_problem = deepcopy(initial_path_problem)
    tentative_path_problem.state_grid = state_grid
    partial_solutions = []

    node_dict = {}
    layout_id = 0

    for connection in completed_set:
        # add fit_pos, get direction of the layout (horizontal/vertical)
        layout_dir = get_direction(diff_pos(connection[0], connection[1]))
        node_dict[(connection[0], layout_id)] = layout_dir
        node_dict[(connection[1], layout_id)] = layout_dir
        layout_id += 1

    exclusion_list = set()  # for keeping track of connections with no valid path

    while True:
        connections = get_best_connections(node_dict=node_dict, exclusion_list=exclusion_list)
        for connection in connections:
            tentative_path_problem.start_pos = connection[0]
            tentative_path_problem.goal_node = connection[1]
            # fixme: also add start_direction, goal_directions depending on trail direction
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


def get_partial_solutions(outgoing_connections_set: set, exclusion_list: list[set],
                          outgoing_directions_dict: dict, state_grid, part_stock, path_problem,
                          ) -> list[Solution]:
    """
    Generates a connection for each two nodes according to the lowest manhattan distance.
    :param node_dict: List containing position of nodes and corresponding layout ids.
    :param exclusion_list: List with connections that are excluded (for example because there is no feasible path in a connection).
    :return: Set of connections
    """
    # todo: add start and goal
    #   get current state grid
    #   get current part stock (from outside this func with completed layouts)
    start = path_problem.start_pos
    goal = path_problem.goal_pos

    partial_solutions = []

    all_points_dict = {}  # reference of outgoing position to its connection

    # get dictionary of all outgoing positions referencing their connection
    for end_points in outgoing_connections_set:
        for point_tuple in end_points:
            if point_tuple == ():
                continue
            all_points_dict[point_tuple] = end_points

    # todo: sort all_points by manhattan distance to start
    # sort all
    all_points_list = sorted(all_points_dict.keys(), key=lambda x: manhattan_distance(start, x))

    while all_points_list:
        open_list = []
        point_pos = all_points_list.pop(0)
        point_outgoing_connections_ref = all_points_dict[point_pos]
        solution = None
        for other_point_pos in all_points_list:
            if other_point_pos in point_outgoing_connections_ref:
                continue
            if {point_pos, other_point_pos} in exclusion_list:
                continue

            partial_path_problem = PathProblem(algorithm=path_problem.algorithm, weights=path_problem.weights,
                                               start_pos=point_pos,
                                               start_directions=outgoing_directions_dict[point_pos],
                                               goal_pos=other_point_pos,
                                               goal_directions=outgoing_directions_dict[other_point_pos],
                                               part_cost=path_problem.part_cost, part_stock=part_stock,
                                               starting_part=0,
                                               state_grid=state_grid)

            solution = get_solution(path_problem=partial_path_problem, draw_debug=False)
            if not solution:
                # try again, but exclude this connection combination
                exclusion_list.append({point_pos, other_point_pos})
                return get_partial_solutions(outgoing_connections_set=outgoing_connections_set,
                                             outgoing_directions_dict=outgoing_directions_dict,
                                             exclusion_list=exclusion_list,
                                             part_stock=part_stock,
                                             path_problem=path_problem,
                                             state_grid=state_grid
                                             )
            heapq.heappush(open_list, (solution.score, other_point_pos))
        # get connecting node with best score, then remove it
        if open_list:
            best_point = heapq.heappop(open_list)[1]
            all_points_list.remove(best_point)
            partial_solutions.append(solution)
            part_stock = solution.part_stock  # adjust part stock for next iteration

    return partial_solutions


def fuse_partial_solutions(partial_solutions: list[Solution], completed_layouts: dict[Trail:BuildingInstruction],
                           initial_path_problem: PathProblem):
    """Fuses partial solutions and completed layouts into a complete solution from start to goal of the initial path problem.
    Only partially checks the validity of the complete solution. Partial solutions and completed layouts must therefore
    be compatible and equal to a valid assembly layout."""
    total_definite_trail = {}

    unordered_layouts = set()
    rendering_dict = {}

    # add solution information from already completed layouts
    for layout_trail, layout_state in completed_layouts.items():

        unordered_layouts.add(layout_trail)  # add for use later

        fit_first = layout_state.required_fit_positions[0]
        fit_last = layout_state.required_fit_positions[1]

        rendering_dict[fit_first] = const.fitting_id
        rendering_dict[layout_trail[1]] = layout_state.part_id
        rendering_dict[fit_last] = const.fitting_id

        for pos in layout_trail:
            if pos in layout_state.required_fit_positions:
                total_definite_trail[pos] = const.fitting_id
            else:
                total_definite_trail[pos] = layout_state.part_id

    # also update total definite trail from partial solutions
    for partial_solution in partial_solutions:
        total_definite_trail.update(partial_solution.total_definite_trail)

    # get information from last partial solution iteration
    last_partial_solution = partial_solutions[0]
    part_stock = last_partial_solution.part_stock
    algorithm = last_partial_solution.algorithm
    state_grid = last_partial_solution.state_grid

    # add partial solution layout_trails to unordered layouts, too
    for partial_solution in partial_solutions:
        for layout_trail in partial_solution.layouts:
            unordered_layouts.add(layout_trail)

    # sort unordered layouts
    ordered_layouts = []

    # search for the first layout
    add_layout = ()
    for layout_trail in unordered_layouts:
        if initial_path_problem.start_pos in layout_trail:
            add_layout = layout_trail
            break

    while add_layout in unordered_layouts:
        unordered_layouts.remove(add_layout)
        ordered_layouts.append(add_layout)
        for other_layout in unordered_layouts:
            if add_layout[-1] == other_layout[0]:
                add_layout = other_layout
                break

    # todo: if needed, recalculate score
    score = last_partial_solution.score
    fused_solution = Solution(part_stock=part_stock, path_problem=initial_path_problem, rendering_dict=rendering_dict,
                              algorithm=algorithm, state_grid=state_grid, score=score, layouts=ordered_layouts,
                              total_definite_trail=total_definite_trail)
    return fused_solution
