from __future__ import annotations

from PathFinding.pf_data_class.Weights import Weights
from PathFinding.util.path_math import manhattan_distance, get_direction
from PathFinding.util.restrictions import out_of_bounds


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


def get_worst_move_cost(part_cost: dict) -> (float, list):
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


def get_f_score(current_score_start_distance: float, current_score_goal_distance: float,
                current_score_extra: float, total_score_current_node: float, algorithm: str) -> float:
    score = current_score_start_distance + current_score_goal_distance
    if algorithm == "mcsa*":
        score = score + current_score_extra + total_score_current_node
    elif algorithm == "mca*":
        score = score + current_score_extra

    return score


def get_m_score(algorithm: str, weights: Weights, neighbor_pos: tuple, goal_pos: tuple, upper_bound: float) -> float:
    """Calculates normalized M Score."""
    # M Score: Distance score to goal node
    score = 0
    if algorithm != "dijkstra":  # dijkstra doesn't include M score
        score = manhattan_distance(neighbor_pos, goal_pos) * weights.path_length / upper_bound

    return score


def get_e_score(algorithm: str, weights: Weights, current_pos: tuple, neighbor_pos: tuple, part_id: int,
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
        cost = ((part_cost[part_id] / length) / worst_move_cost) * weights.cost  # calculates cost per passed nodes

        distance_to_obstacles = calculate_distance_to_obstacles(current_state_grid, current_pos,
                                                                neighbor_pos) * weights.distance_to_obstacles

        score = cost + distance_to_obstacles

    return score
