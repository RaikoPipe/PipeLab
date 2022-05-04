from __future__ import annotations

from copy import deepcopy
from typing import Optional

from PathFinding.path_finding_util.path_math import diff_pos, get_direction, sum_pos
from PathFinding.pf_data_class.Predecessor import Predecessor
from PathFinding.pf_data_class.Solution import Solution
from type_dictionary.common_types import *


def construct_solution(predecessors, current_node, state_grid, score,
                       algorithm, path_problem, fast_mode, goal_pos, goal_part) -> Solution:
    """Returns a solution based on given parameters. """

    absolute_path = []
    rendering_dict = {}
    part_stock = deepcopy(path_problem.part_stock)
    absolute_path.append((goal_pos, goal_part))
    rendering_dict[goal_pos] = goal_part
    part_stock[goal_part] -= 1
    while current_node in predecessors:
        part_id = predecessors.get(current_node).part_to_successor

        if current_node == Pos:  # fixme: doesn't work, fix exists
            absolute_path.append((current_node, part_id))
            rendering_dict[current_node] = part_id
        else:
            absolute_path.append((current_node[0], part_id))
            rendering_dict[current_node[0]] = part_id

        part_stock[part_id] -= 1
        predecessor_node: Predecessor = predecessors.get(current_node)
        if fast_mode:
            current_node = predecessor_node.pos
        else:
            current_node = (predecessor_node.pos, predecessor_node.part_to_predecessor, predecessor_node.direction)
        if current_node is None:
            break

    absolute_path = absolute_path[::-1]  # reverse order

    absolute_trail = {}
    ordered_trails = []
    fc_set = set()
    fit_start_pos = None
    i = 0
    while i < len(absolute_path) - 1:
        start_node = absolute_path[i]

        if start_node[1] == 0 or start_node[1] is None:
            trail_list = []
            absolute_trail[start_node[0]] = start_node[1]
            trail_list.append(start_node[0])
            # get id of straight pipe
            pipe_node = absolute_path[i + 1]
            end_node = absolute_path[i + 2]
            direction = get_direction(diff_pos(start_node[0], end_node[0]))

            pos = start_node[0]
            while pos != end_node[0]:
                pos = (pos[0] + 1 * direction[0], pos[1] + 1 * direction[1])
                # handle transition case
                if state_grid[pos] == 3:
                    continue
                absolute_trail[pos] = pipe_node[1]
                trail_list.append(pos)

            absolute_trail[end_node[0]] = end_node[1]
            ordered_trails.append(tuple(trail_list))
            fc_set.add((start_node[0], end_node[0]))

            i += 2

        else:
            # definite path must start with None or 0
            raise Exception

    return Solution(absolute_path=absolute_path, node_trail=absolute_trail,
                    ordered_trails=ordered_trails, state_grid=state_grid, score=score, algorithm=algorithm,
                    path_problem=path_problem, part_stock=part_stock,
                    rendering_dict=rendering_dict)


def get_corner_neighbors(axis: tuple, available_parts: list) -> set:
    """Returns all the neighbors that are allowed as the next move by the currently available corner parts."""

    # corner moves can ONLY go in one direction
    neighbors = set()
    if 0 in available_parts:
        neighbors.add((axis, 0))

    return neighbors


def get_pipe_neighbors(axis, available_parts, at_start, transition) -> set:
    """Returns all neighbors that are allowed as the next move by the currently available pipe parts"""

    # pipe moves have two variants, depending on the current axis

    neighbors = set()
    for part_id in available_parts:
        # for all part IDs except corner the point length matches the id
        if part_id == 0:
            continue

        if transition:
            neighbors.add(((axis[0] * (part_id + axis[0]), (axis[1] * (part_id + axis[1]))), part_id))
            if not at_start:
                neighbors.add(((part_id * axis[1], part_id * axis[0]), part_id))
                neighbors.add(((part_id * -axis[1], part_id * -axis[0]), part_id))
        elif at_start:
            # only allow neighbors that meet start condition
            neighbors.add(((part_id * axis[0], part_id * axis[1]), part_id))
        else:
            # only allow neighbors that meet corner condition
            neighbors.add(((part_id * axis[1], part_id * axis[0]), part_id))
            neighbors.add(((part_id * -axis[1], part_id * -axis[0]), part_id))

    return neighbors


def get_changed_nodes(predecessor_pos, current_pos):
    pos = diff_pos(predecessor_pos, current_pos)

    direction = get_direction(pos)
    length = abs(pos[0] - pos[1])

    occupied_nodes = []

    for i in range(1, length + 1):
        pos = (predecessor_pos[0] + direction[0] * i, predecessor_pos[1] + direction[1] * i)
        occupied_nodes.append((pos, 2))

    return occupied_nodes


def build_path(current_node: tuple, predecessor: dict, start_node: tuple) -> list:
    """Constructs a path from start to the current node."""

    path = []

    while current_node in predecessor:
        path.append(current_node)
        current_node = predecessor.get(current_node).predecessor_node
    path.append(start_node)
    path = path[::-1]  # reverses the path to correct order (from start to goal)
    return path


def get_current_state_grid(current_path, state_grid):
    """Returns the current state grid according to the current path."""
    current_state_grid = deepcopy(state_grid)
    for index, v in enumerate(current_path):
        if index == len(current_path) - 1:
            break
        a = current_path[index]
        b = current_path[index + 1]
        pos = (b[0] - a[0], b[1] - a[1])
        direction = get_direction(pos)
        length = abs(pos[0] - pos[1])
        for i in range(1, length + 1):
            pos = (a[0] + direction[0] * i, a[1] + direction[1] * i)
            current_state_grid[pos] = 2  # 2: occupied by pipe
    return current_state_grid


def pipe_stock_check(pipe_stock: dict, predecessors: dict, fast_mode, key) -> list:
    """Checks how many parts are available."""
    available_parts = []
    pipe_stock_copy = deepcopy(pipe_stock)
    if len(predecessors) < 2:
        # no parts used, no need to check current path
        available_parts = get_available_parts(pipe_stock_copy)
        return available_parts

    while key in predecessors:
        part_id = predecessors.get(key).part_to_successor

        predecessor_node: Predecessor = predecessors.get(key)

        if fast_mode:
            key = predecessor_node.pos
        else:
            key = (predecessor_node.pos, predecessor_node.part_to_predecessor, predecessor_node.direction)

        pipe_stock_copy[part_id] -= 1

        available_parts = get_available_parts(pipe_stock_copy)

    return available_parts


def get_available_parts(pipe_stock: dict) -> list:
    """Returns available parts as list."""
    available_parts = []
    for _, (part_id, amount) in enumerate(pipe_stock.items()):
        if amount > 0:
            if part_id in available_parts:
                continue
            else:
                available_parts.append(part_id)
    return available_parts


def is_between(a: Pos, b: Pos, pos: Pos) -> bool:
    # todo: change frozenset to tuple, because iterating through frozenset might be slow

    fit_dir = get_direction(diff_pos(a, b))
    con_dir = get_direction(diff_pos(pos, a))

    return fit_dir == con_dir


def get_transition(pos, direction, transition_points) -> Optional[tuple]:
    for transition_point in transition_points:
        check_pos = sum_pos(pos, direction)
        if check_pos[0] == transition_point[0] or check_pos[1] == transition_point[1]:
            transition = (transition_point, direction)
            return transition

    return None
