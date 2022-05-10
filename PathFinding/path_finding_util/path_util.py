from __future__ import annotations

from copy import deepcopy
from typing import Optional

from PathFinding.path_finding_util.path_math import diff_pos, get_direction, sum_pos
from PathFinding.pf_data_class.PathProblem import PathProblem
from PathFinding.pf_data_class.Predecessor import Predecessor
from PathFinding.pf_data_class.Solution import Solution

from TypeDictionary.class_types import *


def construct_solution(predecessors: Predecessors, current_node: Union[Pos, tuple[Pos, int, Pos]],
                       state_grid: StateGrid, score: float,
                       algorithm: str, path_problem: PathProblem, fast_mode: bool, goal_pos: Pos,
                       goal_part: int) -> Solution:
    """Constructs a solution based on given parameters.

    Args:
        state_grid(:obj`~type_aliases.State`): State grid with applied solution path
        current_node(:obj`~type_aliases.Pos`): Last visited node in search.
        predecessors(:obj:`~special_types.Predecessors`): All visited predecessors.
        score(:obj`float`): The final solution score.
        algorithm(:obj`str`): Algorithm used in prior search.
        path_problem(:class:`PathProblem`)
        fast_mode(:obj`bool`): See :paramref:`~search_algorithm.find_path.fast_mode`
        goal_pos(:obj`~type_aliases.Pos`): The reached goal position.
        goal_part(:obj`int`): Part that should be appended to reach actual goal position.

    Returns:
        :class:`Solution`
        """

    node_path: NodePath = []
    rendering_dict: RenderingDict = {}
    part_stock = deepcopy(path_problem.part_stock)
    node_path.append((goal_pos, goal_part))
    rendering_dict[goal_pos] = goal_part
    part_stock[goal_part] -= 1
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

    absolute_trail = {}
    ordered_trails = []
    fc_set = set()
    fit_start_pos = None
    i = 0
    while i < len(node_path) - 1:
        start_node = node_path[i]

        if start_node[1] == 0 or start_node[1] is None:
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

    return Solution(node_path=node_path, node_trail=absolute_trail,
                    ordered_trails=tuple(ordered_trails), state_grid=state_grid, score=score, algorithm=algorithm,
                    path_problem=path_problem, part_stock=part_stock,
                    rendering_dict=rendering_dict)


def get_corner_neighbors(direction: Pos, available_parts: set[int]) -> set[Node]:
    """Returns all the neighbors that are allowed as the next move by the currently available corner parts.

    Args:
        direction(:obj:`~type_aliases.Pos`): Considered direction for pipe
        available_parts(:obj:list [:obj:`int`]):
    """

    # corner moves can ONLY go in one direction
    neighbors = set()
    if 0 in available_parts:
        neighbors.add((direction, 0))

    return neighbors


def get_pipe_neighbors(axis: Pos, available_parts: set[int], at_start: bool, transition: tuple[Pos, Pos]) -> set[Node]:
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


def get_changed_nodes(predecessor_pos: Pos, current_pos: Pos) -> tuple[Node]:
    """Returns all nodes that have changed after a move.

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

    Returns:
        :obj:`set` containing the part IDs (:obj:`int`) left in stock.

    """
    available_parts = set()
    pipe_stock_copy = deepcopy(part_stock)
    if len(predecessors) < 2:
        # no parts used, no need to check current path
        available_parts = get_available_parts(pipe_stock_copy)
    else:

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


def get_available_parts(pipe_stock: PartStock) -> set[int]:
    """Returns available parts as set.

    Returns:
        :obj:`set` [:obj:`int`]
    """
    available_parts = set()
    for part_id, amount in pipe_stock.items():
        if amount > 0:
            if part_id in available_parts:
                continue
            else:
                available_parts.add(part_id)
    return available_parts


def get_transition(pos, direction, transition_points) -> Optional[tuple[Pos, Pos]]:
    """Checks if transition points lie in the node in direction of the given node position.

    Returns:
        :obj:`tuple` [:obj:`~type_aliases.Pos`,:obj:`~type_aliases.Pos`] if transition point found, else None.
    """
    for transition_point in transition_points:
        check_pos = sum_pos(pos, direction)
        if check_pos[0] == transition_point[0] or check_pos[1] == transition_point[1]:
            transition = (transition_point, direction)
            return transition

    return None
