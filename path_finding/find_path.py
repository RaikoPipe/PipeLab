import heapq
import numpy as np
from datetime import datetime
from path_finding import interpret_path as pint
from vpython import *

from path_finding.path_data_classes import Solution, PathProblem, Weights
from rendering import object_classes, positional_functions_old as lvf
from copy import deepcopy
import time
from path_finding.tuple_math import *
from typing import Optional


# todo: full refactoring
# save showcase inside a show_case_grid

# if neighbor vn is outside array shape, dont allow
def out_of_bounds(neighbor_node: tuple, state_grid):
    """Checks if neighbor is outside of the boundaries of the state grid."""
    if 0 <= neighbor_node[0] < state_grid.shape[0]:
        if 0 <= neighbor_node[1] < state_grid.shape[1]:
            return False
        else:
            return True  # array bound y walls
    else:
        return True  # array bound x walls


def collided_obstacle(current_node: tuple, neighbor_node: tuple, state_grid) -> bool:
    """Checks if the path from current_node to neighbor_node obstructs any obstacles."""

    length = abs(neighbor_node[0] - neighbor_node[1])
    axis = pint.getAxis(neighbor_node)

    for i in range(1, length + 1):
        pos = (current_node[0] + axis[0] * i, current_node[1] + axis[1] * i)
        if state_grid[pos] != 0:
            return True


def build_path(current_node: tuple, predecessor_node: dict, start_node: tuple) -> list:
    """Constructs a path from start to the current node."""

    path = []

    while current_node in predecessor_node:
        path.append(current_node)
        current_node = predecessor_node[current_node]
    path.append(start_node)
    path = path[::-1]  # reverses the path to correct order (from start to goal)
    return path


def manhattan_distance(a, b):
    """Calculates the distance between to nodes in horizontal/vertical steps required."""
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    return distance


def calculate_distance_to_obstacles(state_grid, current_node: tuple, neighbor_pos: tuple) -> float:
    """Calculates the amount of obstacles next to the move divided by the maximum possible amount of obstacles next to
     the move."""

    axis = pint.getAxis(neighbor_pos)
    length = abs(neighbor_pos[0] - neighbor_pos[1])
    min_o = length * 2
    upper_bound = min_o

    relative_right = (-axis[1], -axis[0])
    relative_left = (axis[1], axis[0])

    # check positions next to the move
    min_o = min_o - get_min_o_reduction(relative_direction=relative_right, current_node=current_node, axis=axis,
                                        length=length,
                                        state_grid=state_grid)

    min_o = min_o - get_min_o_reduction(relative_direction=relative_left, current_node=current_node, axis=axis,
                                        length=length,
                                        state_grid=state_grid)

    return min_o / upper_bound  # warning: only python 3 produces a float by dividing two integers


def get_min_o_reduction(current_node, length: int, relative_direction: tuple, axis: tuple, state_grid) -> int:
    """Counts free positions next to the move."""

    reduction = 0
    for i in range(length):  # check pos right of pipe
        n_relative_direction = (relative_direction[0] + (axis[0] * i), relative_direction[1] + (axis[1] * i))
        b_relative_direction = (current_node[0] + n_relative_direction[0], current_node[1] + n_relative_direction[1])
        if not out_of_bounds(b_relative_direction, state_grid):
            if state_grid[b_relative_direction] != 0:
                continue
        reduction = reduction + 1
    return reduction


def neighbor_restricted(current_node, neighbor_node, pos, current_state_grid) -> bool:
    """Contains restriction set that checks if neighbor is restricted."""
    if not out_of_bounds(neighbor_node, current_state_grid):
        if collided_obstacle(current_node, pos, current_state_grid):
            return True
    else:
        return True

    return False


def get_m_score(algorithm: str, weights: Weights, neighbor_node: tuple, goal_node: tuple, upper_bound: float) -> float:
    """Calculates normalized M Score."""
    # M Score: Distance score to goal node
    score = 0
    if algorithm != "dijkstra":  # dijkstra doesn't include M score
        score = manhattan_distance(neighbor_node, goal_node) * weights.path_length / upper_bound

    return score


def get_e_score(algorithm: str, weights: Weights, current_node: tuple, neighbor_pos: tuple, part_id: int,
                part_cost: dict, worst_move_cost: float, current_state_grid) -> float:
    """Calculates normalized E Score for MCA*/MCSA*."""
    # E Score: Extra Score (additional score values)
    score = 0

    # todo: suggestion: add a dict containing part id:length to explicitly differentiate between length and id
    if algorithm == "mca*":
        if part_id == 0:
            length = 1
        else:
            length = part_id
        cost = ((part_cost[part_id]/length) / worst_move_cost) * weights.cost  # calculates cost per passed nodes

        distance_to_obstacles = calculate_distance_to_obstacles(current_state_grid, current_node,
                                                                neighbor_pos) * weights.distance_to_obstacles

        score = cost + distance_to_obstacles

    return score


def get_corner_neighbors(axis: tuple, available_parts: dict) -> list:
    """Returns all the neighbors that are allowed as the next move by the currently available corner parts
     (usually only 1 neighbor)."""

    # corner moves can ONLY go in one direction
    neighbors = []
    if 0 in available_parts:
        neighbors.append((axis, 0))

    return neighbors


def get_pipe_neighbors(axis, available_parts, at_start) -> list:
    """Returns all neighbors that are allowed as the next move by the currently available pipe parts"""

    # pipe moves have two variants, depending on the current axis

    neighbors = []
    for part_id in available_parts:
        # for all part IDs except corner the point length matches the id
        if part_id == 0:
            continue
        if at_start:
            # only allow neighbors that meet start condition
            neighbors.append(((part_id * axis[0], part_id * axis[1]), part_id))
        else:
            # only allow neighbors that meet corner condition
            neighbors.append(((part_id * axis[1], part_id * axis[0]), part_id))
            neighbors.append(((part_id * -axis[1], part_id * -axis[0]), part_id))

    return neighbors


def goal_restricted(part_id, neighbor_node, axis, goal_node, goal_axis):
    """Checks if the axis restriction of the goal is violated."""

    if neighbor_node != goal_node:
        # no restriction check necessary, neighbor doesnt reach goal
        return False

    if part_id == 0:
        # check if corner axis is adjacent to goal axis
        if sum_absolute_a_b(axis[0], goal_axis[1]) != 0 or sum_absolute_a_b(axis[1], goal_axis[0]) != 0:
            return False
    else:
        # simply check if pipe axis is the same as goal axis
        if axis == goal_axis:
            return False

    return True


def determine_neighbor_pos(axis: tuple, goal_node: tuple, goal_axis: tuple, current_node: tuple, current_path,
                           pipe_stock, used_parts) -> list:
    """Determines what neighbors are reachable from the current position and with the available parts."""

    available_parts = pint.pipe_stock_check(current_path, pipe_stock, used_parts)
    neighbor_pos = []
    previous_part = used_parts.get(current_node)

    if previous_part is None:
        # todo: it is assumed that if there is no previous part, we are at start pos. However, this only works when
        #  nodes cant be overwritten while searching; proposal: check instead if current node is start (easy)
        # current node is start -> only one axis is allowed
        neighbor_pos.extend(get_corner_neighbors(axis, available_parts))
        neighbor_pos.extend(get_pipe_neighbors(axis, available_parts, True))
    elif previous_part == 0:
        # previous move was corner -> only pipes allowed
        neighbor_pos.extend(get_pipe_neighbors(axis, available_parts, False))
    else:
        # previous mode was pipe -> only corners allowed
        neighbor_pos.extend(get_corner_neighbors(axis, available_parts))

    for idx, (pos, part_id) in enumerate(neighbor_pos):
        if goal_restricted(part_id=part_id, neighbor_node=sum_tuple(current_node, pos), axis=axis, goal_node=goal_node,
                           goal_axis=goal_axis):
            # disallow any neighbors that violate goal condition
            neighbor_pos.pop(idx)

    return neighbor_pos


def get_f_score(current_score_start_distance: float, current_score_goal_distance: float,
                current_score_extra: float, total_score_current_node: float, algorithm: str) -> float:
    score = current_score_start_distance + current_score_goal_distance
    if algorithm == "mcsa*":
        score = score + current_score_extra + total_score_current_node
    elif algorithm == "mca*":
        score = score + current_score_extra

    return score


def get_worst_move_cost(part_cost: dict):
    # is there a more efficient way of doing this?
    worst_move_cost = 0
    for _, (part_id, cost) in enumerate(part_cost.items()):
        length = part_id
        if part_id == 0:
            length = 1
        if worst_move_cost < cost / length:
            worst_move_cost = cost / length
    return worst_move_cost


class PathFinder:
    """class for calculating a path solution"""

    def __init__(self, path_problem: PathProblem):
        self.path_problem = path_problem
        self.state_grid = path_problem.state_grid
        self.start_node = path_problem.start_node
        self.goal_node = path_problem.goal_node
        self.start_axis = path_problem.start_axis
        self.goal_axis = path_problem.goal_axis
        self.pipe_stock = path_problem.pipe_stock
        self.goal_is_transition = path_problem.goal_is_transition
        self.part_cost = path_problem.part_cost
        self.worst_move_cost = get_worst_move_cost(self.part_cost)

    def find_path(self, weights, algorithm) -> Optional[Solution]:
        """Searches for a solution for the given path problem."""

        closed_list = set()
        open_list = []

        predecessor_node = {}

        score_start = {self.start_node: 0}  # G
        upper_bound_distance = manhattan_distance(self.start_node,
                                                  self.goal_node)  # artificial upper bound for distance
        total_score = {self.start_node: upper_bound_distance}  # F

        heapq.heappush(open_list, (total_score[self.start_node], self.start_node))

        used_part = {}

        while open_list:
            current_node = heapq.heappop(open_list)[1]  # pops the node with the smallest score from open_list
            current_path = build_path(current_node=current_node, predecessor_node=predecessor_node,
                                      start_node=self.start_node)

            if current_node == self.start_node:
                verifiable_neighbors = determine_neighbor_pos(axis=self.start_axis,
                                                              goal_node=self.goal_node, goal_axis=self.goal_axis,
                                                              current_node=current_node,
                                                              current_path=current_path,
                                                              pipe_stock=self.pipe_stock, used_parts=used_part)
            else:
                axis = pint.getAxis(diff_tuple(current_node, predecessor_node.get(current_node)))
                verifiable_neighbors = determine_neighbor_pos(axis=axis,
                                                              goal_node=self.goal_node, goal_axis=self.goal_axis,
                                                              current_node=current_node,
                                                              current_path=current_path,
                                                              pipe_stock=self.pipe_stock, used_parts=used_part)

            current_state_grid = pint.getAlteredMatrix(current_path, self.state_grid)

            if current_node == self.goal_node:
                # search is finished!
                overall_score = total_score[current_node]
                solution_parts = []
                while current_node in used_part:
                    solution_parts.append(used_part[current_node])
                    current_node = predecessor_node[current_node]
                solution_parts = solution_parts[::-1]

                solution = Solution(current_path, solution_parts, current_state_grid, overall_score, algorithm,
                                    self.path_problem)
                return solution

            closed_list.add(current_node)

            for (pos, part_id) in verifiable_neighbors:
                neighbor_node = sum_tuple(current_node, pos)

                current_score_start_distance = score_start[current_node] + \
                                               manhattan_distance(current_node, neighbor_node) / total_score[
                                                   self.start_node]

                if neighbor_node in closed_list:  # and current_score_start >= score_start.get(neighbor_node, 0):
                    continue

                if neighbor_restricted(current_node=current_node, neighbor_node=neighbor_node, pos=pos,
                                       current_state_grid=current_state_grid):
                    continue

                if current_score_start_distance < score_start.get(neighbor_node, 0) or neighbor_node not in [p[1] for p
                                                                                                             in
                                                                                                             open_list]:
                    predecessor_node[neighbor_node] = current_node

                    used_part[neighbor_node] = part_id

                    score_start[neighbor_node] = current_score_start_distance

                    current_score_goal_distance = get_m_score(algorithm=algorithm, goal_node=self.goal_node,
                                                              neighbor_node=neighbor_node,
                                                              weights=weights, upper_bound=total_score[self.start_node])

                    current_score_extra = get_e_score(algorithm=algorithm, weights=weights, current_node=current_node,
                                                      neighbor_pos=pos,
                                                      part_cost=self.part_cost, worst_move_cost=self.worst_move_cost,
                                                      current_state_grid=current_state_grid, part_id=part_id)

                    total_score[neighbor_node] = get_f_score(current_score_start_distance, current_score_goal_distance,
                                                             current_score_extra, total_score[current_node], algorithm)
                    heapq.heappush(open_list, (total_score[neighbor_node], neighbor_node))
        else:
            # no solution found!
            return None
