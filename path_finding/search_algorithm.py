import heapq
import time
from copy import copy

import matplotlib.pyplot as plt

from path_finding.grid.grid_functions import change_grid_states
from path_finding.path_finding_util.path_math import *
from path_finding.path_finding_util.path_util import *
from path_finding.path_finding_util.restrictions import neighbor_position_restricted, get_restricted_neighbor_nodes
from path_finding.path_finding_util.score_calculation import get_worst_move, get_f_score, get_m_score, \
    get_e_score
from path_finding.pf_data_class.path_problem import PathProblem
from type_dictionary import constants


def plot_path(original_state_grid, node_path, start_pos, goal_pos):
    plot_state_grid = deepcopy(original_state_grid)
    previous_pos = None

    for pos, part_id in node_path:
        if part_id == 0:
            color = 4
        else:
            color = 5
        if previous_pos:
            changed_pos = get_changed_nodes(current_pos=pos, predecessor_pos=previous_pos)
            for c_pos, _ in changed_pos:
                plot_state_grid[c_pos] = color

        previous_pos = pos

    plot_state_grid[start_pos] = -1
    plot_state_grid[goal_pos] = -1

    data = plot_state_grid.tolist()
    if previous_pos:
        plt.text(pos[1], pos[0], str(pos), color="red")
        plt.text(previous_pos[1], previous_pos[0], str(previous_pos), color="red")

    plt.imshow(data)
    plt.show()

    pass


def find_path(path_problem: PathProblem, draw_path: bool = False, fast_mode=False) -> Optional[Solution]:
    """Searches for a solution for the given path problem.

        Args:
            path_problem(:class:`PathProblem<path_problem>`): Path problem to solve.
            fast_mode(:obj:`bool`): Slightly faster, but incomplete search. Not recommended.
            draw_path(:obj:`bool`): Debug option. If set to True, shows path drawn via `matplotlib.pyplot <https://matplotlib.org/3.5.0/api/_as_gen/matplotlib.pyplot.html>`_ after each node expansion.

        Returns:
            :class:`PathProblem<path_problem>` when a solution was found, else :obj:`None`.


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
    worst_move_cost, worst_moves = get_worst_move(part_cost)
    weights = path_problem.weights
    algorithm = path_problem.algorithm

    # set all possible goal positions
    goal_set = set()
    goal_dict = {}

    for goal_direction in goal_directions:
        g_pos = (goal_pos[0] + goal_direction[0], goal_pos[1] + goal_direction[1])
        goal_set.add(g_pos)
        goal_dict[g_pos] = (-goal_direction[0], -goal_direction[1])

    closed_list = set()
    open_list = []

    key_dict = {1: start_pos, 0: (start_pos, starting_part, None)}

    predecessors = {
        key_dict.get(fast_mode): Predecessor(state_grid=state_grid, direction=None, path=(), part_to_predecessor=None,
                                             part_to_successor=constants.fitting_id,
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
        # pop the node with the smallest score from open_list
        current_node = heapq.heappop(open_list)[1]
        current_pos = current_node[0]
        current_part_id = current_node[1]
        current_direction = current_node[2]

        key_dict[0] = (current_pos, current_part_id, current_direction)
        key_dict[1] = current_pos

        current_path = list(copy(predecessors.get(key_dict.get(fast_mode)).path))
        current_path.append(current_pos)

        if current_pos == start_pos:
            verifiable_neighbors = get_restricted_neighbor_nodes(directions=start_directions,
                                                                 goal_dict=goal_dict,
                                                                 current_pos=current_pos,
                                                                 start_pos=start_pos,
                                                                 transition_points=transition_points,
                                                                 part_stock=pipe_stock, predecessors=predecessors,
                                                                 fast_mode=fast_mode, key=key_dict.get(fast_mode),
                                                                 state_grid=current_state_grid)
        else:
            current_state_grid = change_grid_states(
                state_grid=copy(predecessors.get(key_dict.get(fast_mode)).state_grid),
                node_states=get_changed_nodes(
                    predecessors.get(key_dict.get(fast_mode)).pos, current_pos))

            verifiable_neighbors = get_restricted_neighbor_nodes(directions={current_direction},
                                                                 goal_dict=goal_dict,
                                                                 current_pos=current_pos,
                                                                 start_pos=start_pos,
                                                                 transition_points=transition_points,
                                                                 part_stock=pipe_stock, predecessors=predecessors,
                                                                 fast_mode=fast_mode, key=key_dict.get(fast_mode),
                                                                 state_grid=current_state_grid)

        if draw_path:
            node_path: NodePath = []
            node_path, _ = construct_node_path_and_rendering_dict(current_node=current_node, fast_mode=fast_mode,
                                                                  node_path=node_path,
                                                                  part_stock=path_problem.part_stock,
                                                                  predecessors=predecessors, rendering_dict={})

            plot_path(original_state_grid=path_problem.state_grid, node_path=node_path, start_pos=start_pos,
                      goal_pos=goal_pos)

        if current_pos in goal_set:
            # search is finished!

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

            final_score = total_score[current_pos]
            # current_pos = goal_pos

            return construct_solution(predecessors=predecessors, current_node=current_node,
                                      state_grid=current_state_grid,
                                      score=final_score, goal_pos=goal_pos, goal_part=constants.fitting_id,
                                      algorithm=algorithm, path_problem=path_problem, fast_mode=fast_mode)

        closed_list.add(key_dict.get(fast_mode))

        for (relative_neighbor_pos, neighbor_part_id) in verifiable_neighbors:
            neighbor_pos = sum_pos(relative_neighbor_pos, current_pos)
            neighbor_direction = get_direction(relative_neighbor_pos)
            neighbor_node = (neighbor_pos, neighbor_part_id, neighbor_direction)

            current_score_start_distance = score_start[current_pos] + manhattan_distance(current_pos, neighbor_pos) / \
                                           total_score[start_pos]

            key_dict[0] = (neighbor_pos, neighbor_part_id, neighbor_direction)
            key_dict[1] = neighbor_pos

            if key_dict.get(fast_mode) in closed_list:  # and current_score_start >= score_start.get(neighbor_pos, 0):
                continue

            if neighbor_position_restricted(current_pos=current_pos, neighbor_pos=neighbor_pos,
                                            relative_neighbor_pos=relative_neighbor_pos,
                                            state_grid=current_state_grid, part_id=neighbor_part_id):
                continue

            #p_list = [p[1] for p in open_list]

            current_score_goal_distance = get_m_score(algorithm=algorithm, goal_pos=goal_pos,
                                                      neighbor_pos=neighbor_pos,
                                                      weights=weights, upper_bound=total_score[start_pos])

            current_score_extra = get_e_score(algorithm=algorithm, weights=weights, current_pos=current_pos,
                                              neighbor_pos=neighbor_pos,
                                              part_cost=part_cost, worst_move_cost=worst_move_cost,
                                              state_grid=current_state_grid, part_id=neighbor_part_id)

            current_f_score = get_f_score(current_score_start_distance, current_score_goal_distance,
                                          current_score_extra, total_score[current_pos], algorithm)

            # if (neighbor_pos, neighbor_part_id,
            #     neighbor_direction) not in p_list or current_f_score < total_score.get(neighbor_pos, 0):

            predecessors[key_dict.get(fast_mode)] = Predecessor(pos=current_pos, part_to_successor=neighbor_part_id,
                                                                part_to_predecessor=current_part_id,
                                                                direction=current_direction,
                                                                path=tuple(current_path),
                                                                state_grid=current_state_grid,
                                                                part_stock=pipe_stock)

            score_start[neighbor_pos] = current_score_start_distance

            total_score[neighbor_pos] = get_f_score(current_score_start_distance, current_score_goal_distance,
                                                    current_score_extra, total_score[current_pos], algorithm)

            heapq.heappush(open_list, (total_score[neighbor_pos], neighbor_node))
    else:
        # no solution found!
        return None
