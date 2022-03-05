import numpy as np

from data_class.Weights import Weights
from path_finding.p_math import sum_absolute_a_b, sum_pos, diff_pos
from copy import deepcopy

# todo: put non restriction functions into support functions
from path_finding.path_utilities import get_direction


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

#todo: simplify this
def pipe_stock_check(current_path:list, pipe_stock:dict, used_parts:dict) -> list:
    """Checks how many parts are available."""
    available_parts = []
    pipe_stock_copy = deepcopy(pipe_stock)
    if not used_parts:
        #no parts used, no need to check current path
        available_parts = get_available_parts(pipe_stock_copy)
        return available_parts


    for idx, _ in enumerate(current_path):

        #dont check next point if last point has been reached
        if idx == len(current_path)-1:
            break

        b = current_path[idx + 1]
        og_part = used_parts.get(b)

        pipe_stock_copy[og_part] = pipe_stock_copy[og_part] - 1
        available_parts = get_available_parts(pipe_stock_copy)

    return available_parts


def get_current_state_grid(current_path, state_grid):
    """Returns the current state grid according to the current path."""
    current_state_grid = deepcopy(state_grid)
    for index, v in enumerate(current_path):
        if index == len(current_path) - 1:
            break
        a = current_path[index]
        b = current_path[index + 1]
        pos = (b[0] - a[0],b[1] - a[1])
        direction = get_direction(pos)
        length = abs(pos[0] - pos[1])
        for i in range(1, length + 1):
            pos = (a[0] + direction[0] * i, a[1] + direction[1] * i)
            current_state_grid[pos] = 2 # 2: occupied by pipe
    return current_state_grid


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
    direction = get_direction(neighbor_node)

    for i in range(1, length + 1):
        node = (current_node[0] + direction[0] * i, current_node[1] + direction[1] * i)
        if state_grid[node] != 0:
            return True


def build_path(current_node: tuple, predecessor: dict, start_node: tuple) -> list:
    """Constructs a path from start to the current node."""

    path = []

    while current_node in predecessor:
        path.append(current_node)
        current_node = predecessor.get(current_node).predecessor_node
    path.append(start_node)
    path = path[::-1]  # reverses the path to correct order (from start to goal)
    return path


def manhattan_distance(a:tuple[int,int], b:tuple[int,int]):
    """Calculates the distance between to nodes in horizontal/vertical steps required."""
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    return distance


def calculate_distance_to_obstacles(state_grid, current_node: tuple, neighbor_pos: tuple) -> float:
    """Calculates the amount of obstacles next to the move divided by the maximum possible amount of obstacles next to
     the move."""

    axis = get_direction(neighbor_pos)
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
    # suggestion: add a dict containing part id:length to explicitly differentiate between length and id
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


def get_corner_neighbors(axis: tuple, available_parts: list) -> list:
    """Returns all the neighbors that are allowed as the next move by the currently available corner parts."""

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


def determine_neighbor_pos(direction: tuple, goal_node: tuple, goal_direction: tuple, current_node: tuple, current_path,
                           pipe_stock, used_parts) -> list:
    """Determines what neighbors are reachable from the current position and with the available parts."""

    available_parts = pipe_stock_check(current_path, pipe_stock, used_parts)
    neighbor_pos = []
    previous_part = used_parts.get(current_node)

    if previous_part is None:
        # suggestion: it is assumed that if there is no previous part, we are at start pos. However, this only works when
        #  nodes cant be overwritten while searching; proposal: check instead if current node is start (easy)
        # current node is start -> only one axis is allowed
        neighbor_pos.extend(get_corner_neighbors(direction, available_parts))
        neighbor_pos.extend(get_pipe_neighbors(direction, available_parts, True))
    elif previous_part == 0:
        # previous move was corner -> only pipes allowed
        neighbor_pos.extend(get_pipe_neighbors(direction, available_parts, False))
    else:
        # previous mode was pipe -> only corners allowed
        neighbor_pos.extend(get_corner_neighbors(direction, available_parts))

    for idx, (pos, part_id) in enumerate(neighbor_pos):
        if goal_restricted(part_id=part_id, neighbor_node=sum_pos(current_node, pos), axis=direction, goal_node=goal_node,
                           goal_axis=goal_direction):
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


def get_worst_move_cost(part_cost: dict) -> (float,list):
    """Calculates the cost of the worst move."""
    # is there a more efficient way of doing this?
    worst_move_cost = 0
    worst_moves = []
    for _, (part_id, cost) in enumerate(part_cost.items()):
        length = part_id
        if part_id == 0:
            length = 1
        if worst_move_cost <= cost / length:
            worst_move_cost = cost / length
            worst_moves.append(part_id)
    return worst_move_cost, worst_moves


def get_changed_nodes(predecessor_node, current_node):
    pos = diff_pos(predecessor_node, current_node)

    direction = get_direction(pos)
    length = abs(pos[0] - pos[1])

    occupied_nodes = []

    for i in range(1, length + 1):
        pos = (predecessor_node[0] + direction[0] * i, predecessor_node[1] + direction[1] * i)
        occupied_nodes.append((pos, 2))

    return occupied_nodes