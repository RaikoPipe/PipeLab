from math import copysign

from data_class.Solution import Solution
from path_finding.path_math import diff_pos
from path_finding.restriction_functions import manhattan_distance
from common_types import *

import heapq

# def get_best_solution_from_nodes(path_problem, connecting_nodes):
#     """Returns best solution for each node in connecting_node as goal."""
#     # get scores for connecting to start
#     solutions = []
#     for connecting_node in connecting_nodes:
#         path_problem.goal_node = connecting_node
#         solution = find_path(path_problem)
#         solutions.append(solution)
#         # todo: get axis of connecting node somehow
#
#
#     # todo: get solution with lowest score; use heapq
#     best_solution = Solution() #todo: see line 71
#
#     return best_solution

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


def get_diagonal_direction(pos:tuple) -> tuple:
    """Returns the direction of a relative position."""
    x = pos[0]
    y = pos[1]
    x= int(copysign(x,pos[0]))
    y= int(copysign(y,pos[1]))
    return x,y


def get_direction(pos:tuple) -> tuple:
    """Returns the direction of a relative position."""

    x = pos[0]**0
    y = pos[1]**0

    x= int(copysign(x,pos[0]))
    y= int(copysign(y,pos[1]))

    return x,y

def construct_solution(predecessors, current_node, state_grid, score,
                       algorithm, path_problem):
    definite_path = []
    while current_node in predecessors:
        definite_path.append((current_node, predecessors.get(current_node).part_used))
        current_node = predecessors.get(current_node).pos

    definite_path = definite_path[::-1]  # reverse order

    total_definite_trail = {}
    layouts = []
    fc_set = set()
    fit_start_pos = None
    i = 0
    while i < len(definite_path):
        start_node = definite_path[i]

        if start_node[1] == 0 or start_node[1] is None:
            trail = []
            total_definite_trail[start_node[0]] = start_node[1]
            trail.append(start_node[0])
            #get id of straight pipe
            pipe_node = definite_path[i+1]
            end_node = definite_path[i+2]
            direction = get_direction(diff_pos(start_node, end_node))

            pos = start_node[0]
            while pos != end_node:
                pos = (pos[0]+1*direction[0], pos[1]+1*direction[1])
                total_definite_trail[pos] = pipe_node[1]
                trail.append(pos)
            total_definite_trail[end_node[0]] = end_node[1]
            trail.append(end_node)
            layouts.append(trail)
            fc_set.add((start_node[0], end_node[0]))

        else:
            # definite path must start with None or 0
            raise Exception

    return Solution(definite_path = definite_path, fc_set=fc_set, total_definite_trail=total_definite_trail,
                    layouts=layouts, state_grid = state_grid, score=score, algorithm=algorithm, path_problem=path_problem)
