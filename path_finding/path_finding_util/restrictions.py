from __future__ import annotations

from path_finding.path_finding_util.path_math import sum_pos, get_direction
from path_finding.path_finding_util.path_util import get_corner_neighbors, get_pipe_neighbors, pipe_stock_check, \
    get_transition
from type_dictionary.class_types import Predecessors
from type_dictionary.constants import fitting_id
from type_dictionary.type_aliases import DirectionDict, PosSet, Pos, StateGrid, PartStock, Node


def out_of_bounds(neighbor_pos: Pos, state_grid: StateGrid):
    """Checks if neighbor is outside of the boundaries of the state grid.
    Used in :func:`neighbor_positions_restricted`.

    Args:
        neighbor_pos(:obj:`~type_aliases.Pos`): Node position of considered neighbor node.
        state_grid(:obj:`~type_aliases.StateGrid`): To the current path modified state grid.

    Returns:
        :obj:`bool`
    """
    if 0 <= neighbor_pos[0] < state_grid.shape[0]:
        if 0 <= neighbor_pos[1] < state_grid.shape[1]:
            return False
        else:
            return True  # array bound y walls
    else:
        return True  # array bound x walls


collision_set = {1, 2}


def collided_obstacle(current_pos: Pos, neighbor_pos: Pos, state_grid: StateGrid, part_id: int) -> bool:
    """Checks if the path from current_node to neighbor_node obstructs any obstacles or transitions.
    Used in :func:`neighbor_positions_restricted`.

    Args:
        current_pos(:obj:`~type_aliases.Pos`): Node position of currently expanded node.
        neighbor_pos(:obj:`~type_aliases.Pos`): Node position of considered neighbor node.
        state_grid(:obj:`~type_aliases.StateGrid`): To the current path modified state grid.
        part_id(:obj:`int`): Part ID of the neighbor node.

    Returns:
        :obj:`bool`
    """

    length = abs(neighbor_pos[0] - neighbor_pos[1])
    direction = get_direction(neighbor_pos)
    # collision_nodes = []
    # check collision with obstacles
    for i in range(1, length + 1):
        node = (current_pos[0] + direction[0] * i, current_pos[1] + direction[1] * i)
        # collision_nodes.append(node)
        if state_grid[node] in collision_set:
            return True
        elif state_grid[node] == 3 and ((part_id != fitting_id and i != 1) or part_id == fitting_id):
            return True

    # if part_id != fitting_id and collision_nodes:
    #     collision_nodes.pop(0) # first node is allowed to obstruct transition
    #     # check collision with transitions
    #     for node in collision_nodes:
    #         if state_grid[node] == 3:
    #             return True


def neighbor_position_restricted(current_pos: Pos, neighbor_pos: Pos, relative_neighbor_pos: Pos, state_grid: StateGrid,
                                 part_id: int) -> bool:
    """Checks if neighbor violates positional restrictions. Used in :func:`path_finding_util.search_algorithm.find_path`


    Args:
        current_pos(:obj:`~type_aliases.Pos`): Node position of currently expanded node.
        neighbor_pos(:obj:`~type_aliases.Pos`): Node position of considered neighbor node.
        relative_neighbor_pos(:obj:`~type_aliases.Pos`): Relative node position of neighbor
        state_grid(:obj:`~type_aliases.StateGrid`): To the current path modified state grid.
        part_id(:obj:`int`): Part ID of the neighbor node.

    Returns:
        :obj:`bool`

    """
    if not out_of_bounds(neighbor_pos, state_grid):
        if collided_obstacle(current_pos, relative_neighbor_pos, state_grid, part_id):
            return True
    else:
        return True

    return False


def goal_restricted(part_id: int, pos: Pos, direction, goal_dict: DirectionDict) -> bool:
    """Checks if the direction restriction of the goal is violated.

    Args:
        part_id(:obj:`int`): Part ID of Node.
        pos(:obj:`~type_aliases.Pos`): Node position to check.
        direction(:obj:`~type_aliases.Pos`): Direction of node.
        goal_dict(:obj:`DirectionDict`): See :paramref:`~get_restricted_neighbor_nodes.goal_dict`

    Returns:
        :obj:`bool`"""

    restricted = True

    # if part_id == 0:
    #     # check if corner axis is adjacent to goal axis
    #     if sum_absolute_a_b(direction[0], goal_direction[1]) != 0 or sum_absolute_a_b(direction[1], goal_direction[0]) != 0:
    #         restricted = False
    if part_id == 0:
        return True
    else:
        # simply check if pipe axis is the same as goal axis
        if goal_dict.get(pos) == direction:
            restricted = False

    return restricted


def get_restricted_neighbor_nodes(directions: PosSet, goal_dict: DirectionDict, current_pos: Pos,
                                  part_stock: PartStock, predecessors: Predecessors, fast_mode: bool, key,
                                  start_pos: Pos,
                                  transition_points: set[Pos]) -> set[Node]:
    """Determines what neighbors are reachable from the current position and with the available parts. Removes neighbors
    that violate goal restrictions.

    Args:
        part_stock(:obj:`~type_aliases.PartStock`): See :obj:`~type_aliases.PartStock`
        predecessors(:obj:`~class_types.Predecessors`): See :obj:`~class_types.Predecessors`
        fast_mode(:obj:`bool`): See :paramref:`~pathfinding_util.search_algorithm.find_path.fast_mode`
        key: Key for getting predecessor to the currently expanded node. Type depends on if fast_mode is set.
        directions(:obj:`set` [:obj:`~type_aliases.Pos`]): Directions in which neighbors are considered.
        goal_dict(:obj:`~type_aliases.DirectionDict`): Directions in which the goal can be appended to.
        current_pos(:obj:`~type_aliases.Pos`): The position of the currently expanded node.
        predecessors(:obj:`~class_types.Predecessors`): See :obj:`~class_types.Predecessors`
        start_pos(:obj:`~type_aliases.Pos`): See :obj:`~PathProblem.start_pos`
        transition_points(:obj:`set` [:obj:`~type_aliases.Pos`]): Set of node positions that are marked as transition points.

    Returns:
        A :obj:`set` of restricted neighbor :obj`Nodes<type_aliases.Node>`.

    """

    neighbor_relative_nodes = set()
    previous_part = predecessors.get(key).part_to_successor
    available_parts = pipe_stock_check(part_stock, predecessors, fast_mode, key)
    remove = set()

    for direction in directions:
        if current_pos == start_pos:
            transition = get_transition(current_pos, direction, transition_points)
            # neighbor_pos.extend(get_corner_neighbors(direction, available_parts))
            neighbor_relative_nodes.update(get_pipe_neighbors(direction, available_parts, True, transition))
        elif previous_part == 0:
            transition = get_transition(current_pos, direction, transition_points)
            # previous move was corner -> only pipes allowed
            neighbor_relative_nodes.update(get_pipe_neighbors(direction, available_parts, False, transition))
        else:
            # previous mode was pipe -> only corners allowed
            neighbor_relative_nodes.update(get_corner_neighbors(direction, available_parts))

        for relative_node in neighbor_relative_nodes:

            neighbor_pos = sum_pos(current_pos, relative_node[0])
            if neighbor_pos in goal_dict.keys():
                # restriction check necessary, neighbor reaches goal
                if goal_restricted(part_id=relative_node[1], pos=neighbor_pos,
                                   direction=get_direction(relative_node[0]), goal_dict=goal_dict):
                    # disallow any neighbors that violate goal condition
                    remove.add(relative_node)

    return neighbor_relative_nodes.difference(remove)
