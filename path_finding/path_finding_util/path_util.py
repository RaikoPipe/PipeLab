from __future__ import annotations

from copy import deepcopy
from typing import Optional

from path_finding.path_finding_util import path_math
from path_finding.path_finding_util.path_math import diff_pos, get_direction, sum_pos, manhattan_distance
from path_finding.pf_data_class.path_problem import PathProblem
from path_finding.pf_data_class.solution import Solution
from type_dictionary import constants
from type_dictionary.class_types import *


def construct_solution(predecessors: Predecessors, current_node: Union[Pos, tuple[Pos, int, Pos]],
                       state_grid: StateGrid, score: float,
                       algorithm: str, path_problem: PathProblem, fast_mode: bool, goal_pos: Pos,
                       goal_part: int) -> Solution:
    """Constructs a solution based on given parameters.

    Args:
        state_grid(:obj:`~type_aliases.State`): State grid with applied solution path
        current_node(:obj:`~type_aliases.Pos`): Last visited node in search.
        predecessors(:obj:`~class_types.Predecessors`): All visited predecessors.
        score(:obj:`float`): The final solution score.
        algorithm(:obj:`str`): Algorithm used in prior search.
        path_problem(:class:`PathProblem<path_problem>`)
        fast_mode(:obj:`bool`): See :paramref:`~search_algorithm.find_path.fast_mode`
        goal_pos(:obj:`~type_aliases.Pos`): The reached goal position.
        goal_part(:obj:`int`): Part that should be appended to reach actual goal position.

    Returns:
        :class:`Solution<solution>`
        """

    node_path: NodePath = []
    rendering_dict: RenderingDict = {}
    part_stock = deepcopy(path_problem.part_stock)
    rendering_dict[goal_pos] = goal_part
    part_stock[goal_part] -= 1

    node_path.append((goal_pos, goal_part))
    node_path, rendering_dict = construct_node_path_and_rendering_dict(current_node, fast_mode, node_path, part_stock,
                                                                       predecessors, rendering_dict)

    absolute_trail = {}
    ordered_trails = []
    fc_set = set()
    fit_start_pos = None
    i = 0
    while i < len(node_path) - 1:
        start_node = node_path[i]

        if start_node[1] == constants.fitting_id or start_node[1] is None:
            trail_list = []
            absolute_trail[start_node[0]] = start_node[1]
            trail_list.append(start_node[0])
            # get id of straight pipe
            pipe_node = node_path[i + 1]
            end_node = node_path[i + 2]
            direction = get_direction(diff_pos(start_node[0], end_node[0]))

            pos = start_node[0]
            while pos != end_node[0]:
                pos = (pos[0] + 1 * direction[0], pos[1] + 1 * direction[1])
                # handle transition case
                if state_grid[pos] == constants.transition_state:
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

    return Solution(node_path=node_path, node_trail=absolute_trail,
                    ordered_trails=tuple(ordered_trails), state_grid=state_grid, score=score, algorithm=algorithm,
                    path_problem=path_problem, part_stock=part_stock,
                    rendering_dict=rendering_dict)


def construct_node_path_and_rendering_dict(current_node, fast_mode, node_path, part_stock, predecessors,
                                           rendering_dict):
    while current_node in predecessors:
        part_id = predecessors.get(current_node).part_to_successor

        if fast_mode:
            node_path.append((current_node, part_id))
            rendering_dict[current_node] = part_id
        else:
            node_path.append((current_node[0], part_id))
            rendering_dict[current_node[0]] = part_id

        part_stock[part_id] -= 1
        predecessor_node: Predecessor = predecessors.get(current_node)
        if fast_mode:
            current_node = predecessor_node.pos
        else:
            current_node = (predecessor_node.pos, predecessor_node.part_to_predecessor, predecessor_node.direction)
        if current_node is None:
            break
    node_path = node_path[::-1]  # reverse order
    return node_path, rendering_dict


def get_fitting_neighbors(direction: Pos, available_parts: set[int], transition, start_pos) -> set[Node]:
    """Returns a set containing one neighbor, since after a pipe move a fitting move can only occur in one direction.

    Args:
        transition(:obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`~type_aliases.Pos`]): See :func:`get_transition`
        start_pos(:obj:`~type_aliases.Pos`): See :obj:`~PathProblem.start_pos`
        direction(:obj:`~type_aliases.Pos`): Considered direction for fitting
        available_parts(:obj:list [:obj:`int`]): See :func:`pipe_stock_check`
    Returns:
        A :obj:`set` of neighbor :obj`Nodes<type_aliases.Node>`.
    """

    # fitting moves can ONLY go in one direction
    neighbors = set()

    if 0 in available_parts:
        if transition:
            if not transition[2]:
                # we move towards start and transition, transition is possible
                neighbors.add((sum_pos(direction, direction), 0))
        else:
            neighbors.add((direction, 0))

    return neighbors


def get_pipe_neighbors(direction: Pos, available_parts: set[int], at_start: bool, transition: tuple[Pos, Pos]) -> set[
    Node]:
    """Returns all neighbors that are allowed as the next move by the currently available pipe parts

    Args:
        direction(:obj:`~type_aliases.Pos`): Considered direction for pipe.
        available_parts(:obj:`list` [:obj:`int`]): See :func:`pipe_stock_check`
        at_start(:obj:`bool`): If the current position is the start point.
        transition(:obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`~type_aliases.Pos`]): See :func:`get_transition`
    Returns:
        A :obj:`set` of neighbor :obj:`Nodes<type_aliases.Node>`.
    """

    # pipe moves have two variants, depending on the current axis

    neighbors = set()
    for part_id in available_parts:
        # for all part IDs except corner the point length matches the id
        if part_id == 0:
            continue

        if transition:
            if transition[2]:
                directions = {(direction[1], direction[0]),
                              (-direction[1], -direction[0]),
                              (direction[0], direction[1])}

                for direct in directions:
                    if direct == transition[1]:
                        neighbors.add(
                            ((direct[0] * (part_id + abs(direct[0])), (direct[1] * (part_id + abs(direct[1])))), part_id))
                    elif direction != direct:
                        neighbors.add(((part_id * direct[0], part_id * direct[1]), part_id))

        elif at_start:
            # only allow neighbors that meet start condition
            neighbors.add(((part_id * direction[0], part_id * direction[1]), part_id))
        else:
            # only allow neighbors that meet corner condition
            neighbors.add(((part_id * direction[1], part_id * direction[0]), part_id))
            neighbors.add(((part_id * -direction[1], part_id * -direction[0]), part_id))

    return neighbors


def get_changed_nodes(predecessor_pos: Pos, current_pos: Pos) -> tuple[Node]:
    """Returns all nodes that have changed after a move.

    Args:
        predecessor_pos(:obj:`~type_aliases.Pos`): Node position opf the predecessor to the current node position.
        current_pos(:obj:`~type_aliases.Pos`): The currently expanded node position.

    Returns:
        :obj:`tuple` [:obj:`~type_aliases.Node`]

        """
    pos = diff_pos(predecessor_pos, current_pos)

    direction = get_direction(pos)
    length = abs(pos[0] - pos[1])

    occupied_nodes = []

    for i in range(1, length + 1):
        pos = (predecessor_pos[0] + direction[0] * i, predecessor_pos[1] + direction[1] * i)
        occupied_nodes.append((pos, 2))

    return tuple(occupied_nodes)


def pipe_stock_check(part_stock: PartStock, predecessors: Predecessors, fast_mode: bool,
                     key: Union[Pos, tuple[Pos, int, Pos]]) -> set[int]:
    """Calculates availability of parts.

    Args:
        part_stock(:obj:`~type_aliases.PartStock`): To the current path reduced part stock.
        predecessors(:obj:`~class_types.Predecessors`): See :obj:`~class_types.Predecessors`
        fast_mode(:obj:`bool`): See :paramref:`~pathfinding_util.search_algorithm.find_path.fast_mode`
        key: See :paramref:`~path_finding_util.restrictions.restrict_neighbor_pos.key`


    Returns:
        :obj:`set` containing the part IDs (:obj:`int`) left in stock.

    """

    pipe_stock_copy = deepcopy(part_stock)
    if len(predecessors) >= 2:
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


def get_available_parts(part_stock: PartStock) -> set[int]:
    """Returns available part IDs as set.

    Args:
        part_stock(:obj:`~type_aliases.PartStock`): To the current path reduced part stock.


    Returns:
        :obj:`set` [:obj:`int`]
    """
    available_parts = set()
    for part_id, amount in part_stock.items():
        if amount > 0:
            if part_id in available_parts:
                continue
            else:
                available_parts.add(part_id)
    return available_parts


def get_transition(current_pos: Pos, direction: Pos, transition_points: set[Pos], start_pos) -> Optional[
    tuple[Pos, Pos]]:
    """Checks if transition points lie in the node in direction of the given node position.

    Returns:
        :obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`~type_aliases.Pos`] if transition point found, else None.
    """

    directions = {(direction[1], direction[0]),
                  (-direction[1], -direction[0]),
                  (direction[0], direction[1])}

    for transition_point in transition_points:
        for dir in directions:
            check_idx = 0
            if transition_point[0] < 0:
                check_idx = 1
            path_math.manhattan_distance(current_pos, start_pos)
            dist_to_start = current_pos[check_idx] - start_pos[check_idx]

            man1 = path_math.manhattan_distance(current_pos, start_pos)
            man2 = path_math.manhattan_distance((current_pos[0] + dir[0], current_pos[1] + dir[1]), start_pos)
            away_from_start = False
            if man2 > man1:
                away_from_start = True
            check_pos = sum_pos(current_pos, dir)
            if check_pos[0] == transition_point[0] or check_pos[1] == transition_point[1]:
                transition = (transition_point, dir, away_from_start)
                return transition

    return None


def get_other_node_of_pair(node_pair: NodePair, node_pos: Pos) -> Pos:
    pop_set = set(node_pair)
    pop_set.remove(node_pos)
    other_node = pop_set.pop()
    return other_node
