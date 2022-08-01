from __future__ import annotations

from path_finding.path_finding_util import path_math
from path_finding.path_finding_util.path_math import manhattan_distance, get_direction, diff_pos
from path_finding.path_finding_util.restrictions import out_of_bounds
from path_finding.pf_data_class.weights import Weights
from type_dictionary import constants
from type_dictionary.type_aliases import StateGrid, Pos


def get_min_o_reduction(current_pos: Pos, length: int, relative_direction: Pos, axis: Pos, state_grid) -> int:
    """Counts free positions adjacent to the move.

    Args:
        state_grid(:obj:`~type_aliases.StateGrid`): To the current path modified state grid.
        axis(:obj:`~type_aliases.Pos`): Direction of the move.
        length(:obj:`int`): Node length of the move.
        current_pos(:obj:`~type_aliases.Pos`): Current node position in the search.
        relative_direction(:obj:`~type_aliases.Pos`): Direction of adjacent positions to check.

    Returns:
        :obj:`int` amount of occupied adjacent positions.

    """

    reduction = 0
    for i in range(length):
        n_relative_direction = (relative_direction[0] + (axis[0] * i), relative_direction[1] + (axis[1] * i))
        b_relative_direction = (current_pos[0] + n_relative_direction[0], current_pos[1] + n_relative_direction[1])
        if not out_of_bounds(b_relative_direction, state_grid):
            if state_grid[b_relative_direction] != constants.free_state:
                continue
        reduction = reduction + 1
    return reduction


def calculate_min_o_score(state_grid, current_pos: Pos, neighbor_pos: Pos) -> float:
    """Calculates the amount of obstacles adjacent to the move divided by the maximum possible amount of obstacles
    adjacent to the move.

    Args:
        neighbor_pos(:obj:`~type_aliases.Pos`): Node position of considered neighbor node.
        state_grid(:obj:`~type_aliases.StateGrid`): To the current path modified state grid.
        current_pos(:obj:`~type_aliases.Pos`): Current node position in the search.

    Returns:
        :obj:`float`"""

    axis = get_direction(diff_pos(current_pos, neighbor_pos))
    length = path_math.get_length_same_axis(current_pos, neighbor_pos)
    min_o = length * 2
    upper_bound = min_o

    relative_right = (-axis[1], -axis[0])
    relative_left = (axis[1], axis[0])

    # check positions next to the move
    min_o = min_o - get_min_o_reduction(relative_direction=relative_right, current_pos=current_pos, axis=axis,
                                        length=length,
                                        state_grid=state_grid)

    min_o = min_o - get_min_o_reduction(relative_direction=relative_left, current_pos=current_pos, axis=axis,
                                        length=length,
                                        state_grid=state_grid)

    return min_o / upper_bound  # warning: only python 3 produces a float by dividing two integers


def get_worst_move(part_cost: dict) -> (float, list):
    """Looks for the move with the highest movement cost.

    Args:
        part_cost(:obj:`dict` [:obj:`int`, :obj:`float`]): See :attr:`~path_problem.PathProblem.part_cost`

    Returns:
        :obj:`tuple` containing the per move cost and a list containing the move/s.
    """

    worst_move_cost = 0
    worst_moves = []
    for _, (part_id, cost) in enumerate(part_cost.items()):
        length = part_id
        if part_id == constants.fitting_id:
            length = 1
        if worst_move_cost <= cost / length:
            worst_move_cost = cost / length
            worst_moves.append(part_id)
    return worst_move_cost, worst_moves


def get_f_score(current_score_start_distance: float, current_score_goal_distance: float,
                current_score_extra: float, total_score_current_node: float, algorithm: str) -> float:
    """Calculates the total score of a move according to the used search algorithm.

    Args:
        algorithm(:obj:`str`): See :attr:`~path_problem.PathProblem.algorithm`
        total_score_current_node(:obj:`float`): The total score up until the current position of the search.
        current_score_extra(:obj:`float`): See :func:`get_e_score`
        current_score_start_distance(:obj:`float`): (Normalized) Distance score to the start node
        current_score_goal_distance(:obj:`float`): See :func:`get_m_score`

    Returns:
        :obj:`float`

    """

    score = current_score_start_distance + current_score_goal_distance
    if algorithm == "mcsa*":
        score = score + current_score_extra + total_score_current_node
    elif algorithm == "mca*":
        score = score + current_score_extra

    return score


def get_m_score(algorithm: str, weights: Weights, neighbor_pos: tuple, goal_pos: tuple, upper_bound: float) -> float:
    """Calculates normalized M Score -> Distance to the goal divided by an upper bound score.

    Args:

        goal_pos(:obj:`~type_aliases.Pos`): Goal position.
        neighbor_pos(:obj:`~type_aliases.Pos`): Node position of considered neighbor node.
        weights(:class:`~weights.Weights`): See :class:`~weights.Weights`.
        algorithm(:obj:`str`): See :attr:`~path_problem.PathProblem.algorithm`.
        upper_bound: Score to normalize the resulting distance to goal score.


    Returns:
        :obj:`float`

    """
    # M Score: Distance score to goal node
    score = 0
    if algorithm != constants.dijkstra:  # dijkstra doesn't include M score
        score = manhattan_distance(neighbor_pos, goal_pos) * weights.path_length / upper_bound

    return score


def get_e_score(algorithm: str, weights: Weights, current_pos: Pos, neighbor_pos: Pos, part_id: int,
                part_cost: dict, worst_move_cost: float, state_grid: StateGrid) -> float:
    """Calculates normalized E Score (extra score) for MCA*/MCSA*. E score includes additional costs such as part cost and distance to obstacles.

    Args:
        current_pos(:obj:`~type_aliases.Pos`): Current node position in the search.
        part_id: Part ID of the move that was used.
        part_cost(:obj:`dict` [:obj:`int`, :obj:`float`]): See :attr:`~path_problem.PathProblem.part_cost`
        worst_move_cost(:obj:`float`): See :func:`get_worst_move`
        state_grid(:obj:`~type_aliases.StateGrid`): To the current path modified state grid.
        neighbor_pos(:obj:`~type_aliases.Pos`): Node position of considered neighbor node.
        weights(:class:`~weights.Weights`): See :class:`~weights.Weights`.
        algorithm(:obj:`str`): See :attr:`~path_problem.PathProblem.algorithm`.


    Returns:
        :obj:`float`

    """

    score = 0

    if algorithm in {constants.mca_star, constants.mcsa_star}:
        if part_id == constants.fitting_id:
            length = 1
        else:
            length = part_id
        cost = ((part_cost[part_id] / length) / worst_move_cost) * weights.cost  # calculates cost per passed nodes

        distance_to_obstacles = calculate_min_o_score(state_grid, current_pos,
                                                      neighbor_pos) * weights.distance_to_obstacles

        score = cost + distance_to_obstacles

    return score
