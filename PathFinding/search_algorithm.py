import heapq
from copy import copy
from typing import Optional

import matplotlib.pyplot as plt

from PathFinding.grid.grid_functions import change_grid_states
from PathFinding.path_finding_util.path_math import *
from PathFinding.path_finding_util.path_util import *
from PathFinding.path_finding_util.restrictions import neighbor_restricted, restrict_neighbor_pos
from PathFinding.path_finding_util.score_calculation import get_worst_move_cost, get_f_score, get_m_score, \
    get_e_score
from PathFinding.pf_data_class.PathProblem import PathProblem


# fixme: mca* doesn't work, mcsa* doesn't include extra score
def find_path(path_problem: PathProblem, draw_path: bool = False, fast_mode=False) -> Optional[Solution]:
    """Searches for a solution for the given path problem.

        Args:
            path_problem(:class:`PathProblem`): Path problem to solve.
            fast_mode(:obj:`bool`): Slightly faster, but incomplete search. Not recommended.
            draw_path(:obj:`bool`): Debug option. If set to True, shows path drawn via `matplotlib.pyplot <https://matplotlib.org/3.5.0/api/_as_gen/matplotlib.pyplot.html>`_ after each node expansion.

        Returns:
            :class:`PathProblem` when a solution was found, else :obj:`None`.


    """
    starting_part = path_problem.starting_part
    state_grid = deepcopy(path_problem.state_grid)
    start_pos = path_problem.start_pos
    goal_pos = path_problem.goal_pos
    start_directions = path_problem.start_directions
    goal_directions = path_problem.goal_directions
    transition_points = path_problem.transition_points
    pipe_stock = deepcopy(path_problem.part_stock)
    part_cost = path_problem.part_cost
    worst_move_cost, worst_moves = get_worst_move_cost(part_cost)
    weights = path_problem.weights
    algorithm = path_problem.algorithm

    # set all possible goal positions
    goal_set = set()
    goal_dict = {}

    for goal_direction in goal_directions:
        g_pos = (goal_pos[0] + goal_direction[0], goal_pos[1] + goal_direction[1])
        goal_set.add(g_pos)
        goal_dict[g_pos] = (-goal_direction[0], -goal_direction[1])

    # todo: restrict start and goal: only connectable by pipe

    closed_list = set()
    open_list = []

    key_dict = {1: start_pos, 0: (start_pos, starting_part, None)}

    predecessors = {
        key_dict.get(fast_mode): Predecessor(state_grid=state_grid, direction=None, path=(), part_to_predecessor=None,
                                             part_to_successor=0,
                                             part_stock=path_problem.part_stock, pos=None)}
    current_state_grid = state_grid
    current_state_grid[start_pos] = 2
    current_state_grid[goal_pos] = 2

    score_start = {start_pos: 0}  # G
    upper_bound_distance = manhattan_distance(start_pos,
                                              goal_pos)  # artificial upper bound for distance
    total_score = {start_pos: upper_bound_distance}  # F
    heapq.heappush(open_list, (total_score[start_pos], (start_pos, starting_part, None)))

    while open_list:
        current_node = heapq.heappop(open_list)[1]  # pops the pos with the smallest score from open_list
        current_pos = current_node[0]
        current_part_id = current_node[1]
        current_direction = current_node[2]

        key_dict[0] = (current_pos, current_part_id, current_direction)
        key_dict[1] = current_pos

        current_path = list(copy(predecessors.get(key_dict.get(fast_mode)).path))
        current_path.append(current_pos)
        current_path = tuple(current_path)

        if current_pos == start_pos:
            verifiable_neighbors = restrict_neighbor_pos(directions=start_directions,
                                                         goal_dict=goal_dict,
                                                         current_pos=current_pos,
                                                         start_pos=start_pos,
                                                         transition_points=transition_points,
                                                         pipe_stock=pipe_stock, predecessors=predecessors,
                                                         fast_mode=fast_mode, key=key_dict.get(fast_mode))
        else:
            current_state_grid = change_grid_states(
                state_grid=copy(predecessors.get(key_dict.get(fast_mode)).state_grid),
                node_states=get_changed_nodes(
                    predecessors.get(key_dict.get(fast_mode)).pos, current_pos))

            verifiable_neighbors = restrict_neighbor_pos(directions={current_direction},
                                                         goal_dict=goal_dict,
                                                         current_pos=current_pos,
                                                         start_pos=start_pos,
                                                         transition_points=transition_points,
                                                         pipe_stock=pipe_stock, predecessors=predecessors,
                                                         fast_mode=fast_mode, key=key_dict.get(fast_mode))

        if draw_path:
            data = current_state_grid.tolist()
            plt.imshow(data)
            plt.show()

        if current_pos in goal_set:
            # search is finished!

            current_path = list(current_path)
            current_path.append(goal_pos)
            current_state_grid[goal_pos] = 2

            key_dict[0] = (goal_pos, 0, None)
            key_dict[1] = goal_pos
            goal_node = key_dict.get(fast_mode)

            # add a predecessor that reaches goal
            predecessors[goal_node] = Predecessor(pos=current_pos, part_to_successor=0,
                                                  part_to_predecessor=current_part_id,
                                                  direction=get_direction(diff_pos(current_pos, goal_pos)),
                                                  path=tuple(current_path),
                                                  state_grid=current_state_grid, part_stock=pipe_stock)

            end_score = total_score[current_pos]
            # current_pos = goal_pos

            return construct_solution(predecessors=predecessors, current_node=current_node,
                                      state_grid=current_state_grid,
                                      score=end_score, goal_pos=goal_pos, goal_part=0,
                                      algorithm=algorithm, path_problem=path_problem, fast_mode=fast_mode)

        closed_list.add(key_dict.get(fast_mode))

        for (neighbor, neighbor_part_id) in verifiable_neighbors:
            neighbor_pos = sum_pos(neighbor, current_pos)
            neighbor_direction = get_direction(neighbor)
            neighbor_node = (neighbor_pos, neighbor_part_id, neighbor_direction)

            current_score_start_distance = score_start[current_pos] + \
                                           manhattan_distance(current_pos, neighbor_pos) / total_score[
                                               start_pos]

            key_dict[0] = (neighbor_pos, neighbor_part_id, neighbor_direction)
            key_dict[1] = neighbor_pos

            if key_dict.get(fast_mode) in closed_list:  # and current_score_start >= score_start.get(neighbor_pos, 0):
                continue

            if neighbor_restricted(current_node=current_pos, neighbor_node=neighbor_pos, pos=neighbor,
                                   current_state_grid=current_state_grid, part_id=neighbor_part_id):
                continue

            p_list = [p[1] for p in open_list]

            if current_score_start_distance < score_start.get(neighbor_pos, 0) or (
                    neighbor_pos, neighbor_part_id, neighbor_direction) not in p_list:
                predecessors[key_dict.get(fast_mode)] = Predecessor(pos=current_pos, part_to_successor=neighbor_part_id,
                                                                    part_to_predecessor=current_part_id,
                                                                    direction=current_direction, path=current_path,
                                                                    state_grid=current_state_grid,
                                                                    part_stock=pipe_stock)

                score_start[neighbor_pos] = current_score_start_distance

                current_score_goal_distance = get_m_score(algorithm=algorithm, goal_pos=goal_pos,
                                                          neighbor_pos=neighbor_pos,
                                                          weights=weights, upper_bound=total_score[start_pos])

                current_score_extra = get_e_score(algorithm=algorithm, weights=weights, current_pos=current_pos,
                                                  neighbor_pos=neighbor_pos,
                                                  part_cost=part_cost, worst_move_cost=worst_move_cost,
                                                  current_state_grid=current_state_grid, part_id=neighbor_part_id)

                total_score[neighbor_pos] = get_f_score(current_score_start_distance, current_score_goal_distance,
                                                        current_score_extra, total_score[current_pos], algorithm)
                heapq.heappush(open_list, (total_score[neighbor_pos], neighbor_node))
    else:
        # no solution found!
        return None
