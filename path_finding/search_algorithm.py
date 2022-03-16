import heapq

import path_finding.path_utilities
from path_finding import restriction_functions as rest
from data_class.PredecessorData import PredecessorData
from data_class.Solution import Solution
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from path_finding.restriction_functions import manhattan_distance, neighbor_restricted, get_m_score, get_e_score, \
    determine_neighbor_pos, get_f_score, get_worst_move_cost
from path_finding.path_math import *
from typing import Optional
from copy import copy
from grid.grid_functions import change_grid_states

# idea: make a separate function for search showcase
# optimization idea: save pipe stock in each node, save state grid in each node (reduce pipe stock, occupy path from current node to neighbor node on state grid)
# todo: assign current pipe stock to predecessor, use pipe stock of predecessor to determine current pipe stock
# todo: allow start_axis/goal_axis as None to signify that start/goal direction restriction can be ignored

# fixme: solution should always start and end with a pipe ( as if start and end are pipes that need to be connected to)

def find_path(path_problem: PathProblem) -> Optional[Solution]:
    """Searches for a solution for the given path problem."""
    starting_part = path_problem.starting_part
    state_grid = path_problem.state_grid
    start_node = path_problem.start_node
    goal_node = path_problem.goal_node
    start_axis = path_problem.start_direction
    goal_axis = path_problem.goal_direction
    pipe_stock = path_problem.part_stock
    goal_is_transition = path_problem.goal_is_transition
    part_cost = path_problem.part_cost
    worst_move_cost, worst_moves = get_worst_move_cost(part_cost)
    weights = path_problem.weights
    algorithm = path_problem.algorithm



    closed_list = set()
    open_list = []

    predecessor = {
        start_node: PredecessorData(state_grid=state_grid, direction=None, path=[], part_used=starting_part,
                                         node=None, part_stock=path_problem.part_stock)}

    score_start = {start_node: 0}  # G
    upper_bound_distance = manhattan_distance(start_node,
                                              goal_node)  # artificial upper bound for distance
    total_score = {start_node: upper_bound_distance}  # F
    heapq.heappush(open_list, (total_score[start_node], start_node))

    used_part = {}

    while open_list:
        current_node = heapq.heappop(open_list)[1]  # pops the node with the smallest score from open_list
        current_path = copy(predecessor.get(current_node).path)
        current_path.append(current_node)

        if current_node == start_node:
            current_state_grid = state_grid

            verifiable_neighbors = determine_neighbor_pos(direction=start_axis,
                                                          goal_node=goal_node, goal_direction=goal_axis,
                                                          current_node=current_node,
                                                          current_path=current_path,
                                                          pipe_stock=pipe_stock, used_parts=used_part)
        else:
            current_state_grid = change_grid_states(state_grid=copy(predecessor.get(current_node).state_grid),
                                                    node_states=rest.get_changed_nodes(
                                                        predecessor.get(current_node).node, current_node))

            verifiable_neighbors = determine_neighbor_pos(direction=predecessor.get(current_node).direction,
                                                          goal_node=goal_node, goal_direction=goal_axis,
                                                          current_node=current_node,
                                                          current_path=current_path,
                                                          pipe_stock=pipe_stock, used_parts=used_part)


        if current_node == goal_node:
            # search is finished!
            # todo: save definite path, total definite trail, layouts

            overall_score = total_score[current_node]
            solution_parts = []

            while current_node in predecessor:
                solution_parts.append(predecessor.get(current_node).part_used)
                current_node = predecessor.get(current_node).node
            solution_parts = solution_parts[::-1] # reverse order
            solution = Solution(path=current_path, parts=solution_parts, solution_grid=current_state_grid, score=overall_score,
                                algorithm = algorithm, path_problem=path_problem, problem_solved=True)

            return solution

        closed_list.add(current_node)

        for (pos, part_id) in verifiable_neighbors:
            neighbor_node = sum_pos(current_node, pos)

            current_score_start_distance = score_start[current_node] + \
                                           manhattan_distance(current_node, neighbor_node) / total_score[
                                               start_node]

            if neighbor_node in closed_list:  # and current_score_start >= score_start.get(neighbor_node, 0):
                continue

            if neighbor_restricted(current_node=current_node, neighbor_node=neighbor_node, pos=pos,
                                   current_state_grid=current_state_grid):
                continue

            if current_score_start_distance < score_start.get(neighbor_node, 0) or neighbor_node not in [p[1] for p
                                                                                                         in
                                                                                                         open_list]:
                predecessor[neighbor_node] = PredecessorData(node=current_node, part_used=part_id,
                                                             direction=path_finding.path_utilities.get_direction(pos), path=current_path,
                                                             state_grid=current_state_grid, part_stock=pipe_stock)

                used_part[neighbor_node] = part_id

                score_start[neighbor_node] = current_score_start_distance

                current_score_goal_distance = get_m_score(algorithm=algorithm, goal_node=goal_node,
                                                          neighbor_node=neighbor_node,
                                                          weights=weights, upper_bound=total_score[start_node])

                current_score_extra = get_e_score(algorithm=algorithm, weights=weights, current_node=current_node,
                                                  neighbor_pos=pos,
                                                  part_cost=part_cost, worst_move_cost=worst_move_cost,
                                                  current_state_grid=current_state_grid, part_id=part_id)

                total_score[neighbor_node] = get_f_score(current_score_start_distance, current_score_goal_distance,
                                                         current_score_extra, total_score[current_node], algorithm)
                heapq.heappush(open_list, (total_score[neighbor_node], neighbor_node))
    else:
        # no solution found!
        return None


