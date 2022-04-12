from copy import deepcopy

from data_class.Solution import Solution
from path_finding.path_math import diff_pos, manhattan_distance, get_direction
from path_finding.common_types import *

import heapq

def get_outgoing_pos(paths: list[Path], first_pos: Pos, last_pos: Pos) -> set[tuple[Pos, int]]:
    """
    :param paths: List with paths
    :param first_pos: Position of Node that should be connected first.
    :param last_pos: Position of Node that should be connected last.
    :return: List containing all outgoing nodes with their layout indices from each path.
    """
    #add each first/last node into a list+
    pos_id_set = set()
    layout_id = 0
    pos_id_set.add((first_pos, layout_id))
    for layout_path in paths:
        layout_id += 1
        pos_id_set.add((layout_path[0][0], layout_id))
        pos_id_set.add((layout_path[-1][0], layout_id))
    layout_id += 1
    pos_id_set.add((last_pos, layout_id))
    return pos_id_set


def get_connections(outgoing_connections_set : set, exclusion_list: set[DirectedConnection]) \
        -> list[DirectedConnection]:
    """
    Generates a connection for each two nodes according to the lowest manhattan distance.
    :param node_dict: List containing position of nodes and corresponding layout ids.
    :param exclusion_list: List with connections that are excluded (for example because there is no feasible path in a connection).
    :return: Set of connections
    """

    connections = []

    all_points = {}

    for end_points in outgoing_connections_set:
        if end_points in exclusion_list:
            continue
        for point in end_points:
            all_points[point] = end_points

    while all_points:
        open_list = []
        point = all_points.popitem()
        for other_point in all_points.keys():
            if other_point in point[1]:
                continue
            score = manhattan_distance(point[0], other_point)
            heapq.heappush(open_list, (score, other_point))
        # get connecting node with best score, then remove it
        best_point = heapq.heappop(open_list)
        connection = (point, best_point)
        all_points.pop(best_point)
        connections.append(connection)

    return connections


def get_best_connections(node_dict: dict[tuple[Pos, int]:Pos], exclusion_list: set[DirectedConnection])\
        -> list[DirectedConnection]:
    """
    Generates a connection for each two nodes according to the lowest manhattan distance.
    :param node_dict: List containing position of nodes and corresponding layout ids.
    :param exclusion_list: List with connections that are excluded (for example because there is no feasible path in a connection).
    :return: Set of connections
    """

    connecting_path = []
    for (current_pos,current_id) in node_dict.keys():
        node_dict.remove((current_pos, current_id)) #remove current node
        open_list = []
        for (connecting_pos,connecting_id) in node_dict:
            #prevent layouts from connecting to themselves
            if current_id == connecting_id:
                continue
            elif (current_pos,connecting_pos) in exclusion_list or (connecting_pos, current_pos) in exclusion_list:
                continue

            score = manhattan_distance(current_pos, connecting_pos)
            heapq.heappush(open_list, (score,(connecting_pos, connecting_id)))
        # get connecting node with best score, then remove it
        best_node, best_node_id = heapq.heappop(open_list)
        connection = (current_pos, best_node)
        node_dict.remove((best_node, best_node_id))
        connecting_path.append(connection)

    return connecting_path


def construct_solution(predecessors, current_pos, state_grid, score,
                       algorithm, path_problem):
    definite_path = []
    rendering_dict = {}
    part_stock = deepcopy(path_problem.part_stock)
    while current_pos in predecessors:
        part_id = predecessors.get(current_pos).part_used
        definite_path.append((current_pos, part_id))
        rendering_dict[current_pos] = part_id

        part_stock[part_id] -= 1
        current_pos = predecessors.get(current_pos).pos
        if current_pos is None:
            break


    definite_path = definite_path[::-1]  # reverse order



    total_definite_trail = {}
    layouts = []
    fc_set = set()
    fit_start_pos = None
    i = 0
    while i < len(definite_path)-1:
        start_node = definite_path[i]

        if start_node[1] == 0 or start_node[1] is None:
            trail_list = []
            total_definite_trail[start_node[0]] = start_node[1]
            trail_list.append(start_node[0])
            #get id of straight pipe
            pipe_node = definite_path[i+1]
            end_node = definite_path[i+2]
            direction = get_direction(diff_pos(start_node[0], end_node[0]))

            pos = start_node[0]
            while pos != end_node[0]:
                pos = (pos[0]+1*direction[0], pos[1]+1*direction[1])
                total_definite_trail[pos] = pipe_node[1]
                trail_list.append(pos)

            total_definite_trail[end_node[0]] = end_node[1]
            layouts.append(tuple(trail_list))
            fc_set.add((start_node[0], end_node[0]))

            i+=2

        else:
            # definite path must start with None or 0
            raise Exception


    return Solution(definite_path = definite_path, fc_set=fc_set, total_definite_trail=total_definite_trail,
                    layouts=layouts, state_grid = state_grid, score=score, algorithm=algorithm, path_problem=path_problem, part_stock=part_stock,
                    rendering_dict=rendering_dict)


def get_corner_neighbors(axis: tuple, available_parts: list) -> set:
    """Returns all the neighbors that are allowed as the next move by the currently available corner parts."""

    # corner moves can ONLY go in one direction
    neighbors = set()
    if 0 in available_parts:
        neighbors.add((axis, 0))

    return neighbors


def get_pipe_neighbors(axis, available_parts, at_start) -> set:
    """Returns all neighbors that are allowed as the next move by the currently available pipe parts"""

    # pipe moves have two variants, depending on the current axis

    neighbors = set()
    for part_id in available_parts:
        # for all part IDs except corner the point length matches the id
        if part_id == 0:
            continue
        if at_start:
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
        pos = (b[0] - a[0],b[1] - a[1])
        direction = get_direction(pos)
        length = abs(pos[0] - pos[1])
        for i in range(1, length + 1):
            pos = (a[0] + direction[0] * i, a[1] + direction[1] * i)
            current_state_grid[pos] = 2 # 2: occupied by pipe
    return current_state_grid


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


def is_between(a:Pos, b: Pos, pos:Pos) -> bool:
    #todo: change frozenset to tuple, because iterating through frozenset might be slow

    fit_dir = get_direction(diff_pos(a,b))
    con_dir = get_direction(diff_pos(pos, a))

    return fit_dir == con_dir