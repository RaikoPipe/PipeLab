import heapq
from copy import deepcopy

from path_finding.path_finding_util.path_math import manhattan_distance
from path_finding.pf_data_class.path_problem import PathProblem
from path_finding.pf_data_class.solution import Solution
from path_finding.solution_manager import get_solution
from process_planning.pp_data_class.building_instruction import BuildingInstruction
from type_dictionary import constants as const, constants
from type_dictionary.class_types import BuildingInstructions
from type_dictionary.type_aliases import NodePairSet, FittingDirections, Pos
from type_dictionary.type_aliases import Trail, StateGrid, PartStock

draw_debug = False

def get_partial_solutions(outgoing_node_pairs_set: NodePairSet, exclusion_list: list[set],
                          outgoing_node_directions_dict: FittingDirections, state_grid: StateGrid,
                          part_stock: PartStock,
                          path_problem: PathProblem, search_start_pos: Pos, search_goal_pos: Pos
                          ) -> list[Solution]:
    """
    Tries to generate partial solutions with the given node pairs. Node pairs may be excluded if
    no solution to any other node is found. Node pairs in the same tuple are prevented from connecting.

    Args:

        outgoing_node_pairs_set (:obj:`~type_aliases.NodePairset`): Set of node pairs that represent the end points of a layout
        exclusion_list (:obj:`list` [:obj:`set`]): List of node pairs that are excluded from connecting.
        outgoing_node_directions_dict (:obj:`~type_aliases.FittingDirections`): Dictionary containing the directions each node can be connected to.
        state_grid(:obj:`~type_aliases.StateGrid`): See :obj:`~type_aliases.StateGrid`
        part_stock(:obj:`~type_aliases.PartStock`): See :paramref:`~process_state.ProcessState.part_stock`.
        path_problem(:class:`PathProblem<path_problem>`): See :class:'PathProblem'.

    """

    start = path_problem.start_pos

    partial_solutions = []

    all_points_dict = {}  # reference of outgoing position to its connection

    # get dictionary of all outgoing positions referencing their connection
    for end_points in outgoing_node_pairs_set:
        for point_tuple in end_points:
            if point_tuple == ():
                continue
            all_points_dict[point_tuple] = end_points

    # sort all
    all_points_list = sorted(all_points_dict.keys(), key=lambda x: manhattan_distance(start, x))
    all_points_list.remove(search_goal_pos)
    all_neighbors = set(all_points_list)
    open_list = []
    closed_list = []
    excluded_connections = {}

    for s_pos in all_points_list:
        excluded_connections[s_pos] = set()

    score = {search_start_pos: 0}
    heapq.heappush(open_list, (score[search_start_pos], search_start_pos))
    predecessor = {search_start_pos: search_start_pos}
    solution_dict = {frozenset((search_start_pos,)): path_problem}

    tentative_partial_path_problem = PathProblem(algorithm=path_problem.algorithm, weights=path_problem.weights,
                                                 start_pos=search_start_pos,
                                                 start_directions=outgoing_node_directions_dict[search_start_pos],
                                                 goal_pos=search_goal_pos,
                                                 goal_directions=outgoing_node_directions_dict[search_goal_pos],
                                                 part_cost=path_problem.part_cost, part_stock=part_stock,
                                                 starting_part=constants.fitting_id,
                                                 state_grid=state_grid,
                                                 transition_points=path_problem.transition_points)
    # todo: we need one dictionary that excludes certain neighbors from a search position
    # todo: before trying to search for a solution, check if one exists for this connection in solution_dict
    while open_list:
        current_pos = heapq.heappop(open_list)[1]
        closed_list.append(current_pos)
        neighbors = all_neighbors.difference(excluded_connections[current_pos])

        # get the current state grid
        current_state_grid = solution_dict[frozenset((predecessor[current_pos], current_pos))].state_grid

        if set(all_points_list) == set(closed_list):
            # todo: determine the best performing pos of the predecessor (previous pos?)
            partial_path_problem = adjust_partial_path_problem(current_pos, current_state_grid,
                                                               search_goal_pos, outgoing_node_directions_dict,
                                                               tentative_partial_path_problem)

            solution = get_solution(path_problem=partial_path_problem, draw_debug=draw_debug)
            if solution:
                # search finished! Get all solutions.
                solution_dict[frozenset((current_pos, search_goal_pos))] = solution
                predecessor[search_goal_pos] = current_pos
                # todo: get a list of all solutions (in order)
                solution_list = []
                current_pos = search_goal_pos
                while current_pos in predecessor:
                    solution = solution_dict[frozenset((current_pos, predecessor[current_pos]))]
                    current_pos = predecessor[current_pos]
                    solution_list.append(solution)


                return solution_list
            else:
                excluded_connections[current_pos].add(search_goal_pos)
                continue

        current_node_pair = all_points_dict[current_pos]

        for neighbor in neighbors:
            if neighbor in current_node_pair:
                continue

            # set current partial path problem
            # todo: modify part stock!!!
            partial_path_problem = adjust_partial_path_problem(current_pos, current_state_grid, neighbor,
                                                               outgoing_node_directions_dict,
                                                               tentative_partial_path_problem)

            solution = solution_dict.get(frozenset((current_pos, neighbor)))

            if not solution:
                solution = get_solution(path_problem=partial_path_problem, draw_debug=draw_debug)



            if solution:
                solution_dict[frozenset((current_pos, neighbor))] = solution
                predecessor[neighbor] = current_pos
                # if current_pos == search_start_pos:
                #     heapq.heappush(open_list, (solution.score, current_pos))
                # else:
                neighbor_node_pair = set(all_points_dict[neighbor])
                neighbor_node_pair.remove(neighbor)
                # current_node_pair = set(current_node_pair)
                # current_node_pair.remove(neighbor)
                other_neighbor_node = neighbor_node_pair.pop()
                excluded_connections[other_neighbor_node].add(current_pos)
                heapq.heappush(open_list, (solution.score, other_neighbor_node))

            else:
                excluded_connections[current_pos].add(neighbor)
                closed_list.remove(current_pos)
                heapq.heappush(open_list, (0, current_pos))

    else:
        return None

    # while all_points_list:
    #     solution_list = []
    #     point_pos = all_points_list.pop(0)
    #     point_outgoing_node_pairs_ref = all_points_dict[point_pos]
    #     solution = None
    #     for neighbor in neighbors:
    #         if neighbor in point_outgoing_node_pairs_ref:
    #             continue
    #         if {point_pos, neighbor} in exclusion_list:
    #             continue
    #
    #         partial_path_problem = PathProblem(algorithm=path_problem.algorithm, weights=path_problem.weights,
    #                                            start_pos=point_pos,
    #                                            start_directions=outgoing_node_directions_dict[point_pos],
    #                                            goal_pos=neighbor,
    #                                            goal_directions=outgoing_node_directions_dict[neighbor],
    #                                            part_cost=path_problem.part_cost, part_stock=part_stock,
    #                                            starting_part=0,
    #                                            state_grid=deepcopy(state_grid),
    #                                            transition_points=path_problem.transition_points)
    #
    #         draw_debug = False
    #         solution = get_solution(path_problem=partial_path_problem, draw_debug=draw_debug)
    #
    #         if not solution:
    #             # try again, but exclude this connection combination
    #             exclusion_list.append({point_pos, neighbor})
    #             continue
    #
    #         heapq.heappush(solution_list, (solution.score, neighbor, solution))
    #
    #     # all_points_list.remove(point_pos)
    #     neighbors.remove(point_pos)
    #     # get connecting node with best score, then remove it
    #     if solution_list:
    #         best = heapq.heappop(solution_list)
    #         best_point = best[1]
    #         best_solution = best[2]
    #         exclusion_list.append({point_pos, best_point})
    #         all_points_list.remove(best_point)
    #         neighbors.remove(best_point)
    #
    #         partial_solutions.append(best_solution)
    #         part_stock = solution.part_stock  # adjust part stock for next iteration
    #         # state_grid = solution.state_grid
    #
    # return partial_solutions


def adjust_partial_path_problem(current_pos, current_state_grid, neighbor, outgoing_node_directions_dict,
                                tentative_partial_path_problem):
    partial_path_problem = deepcopy(tentative_partial_path_problem)
    partial_path_problem.start_pos, \
    partial_path_problem.start_directions, \
    partial_path_problem.goal_pos, \
    partial_path_problem.goal_directions, \
    partial_path_problem.state_grid = current_pos, outgoing_node_directions_dict[
        current_pos], neighbor,  outgoing_node_directions_dict[neighbor], current_state_grid
    return partial_path_problem


# def get_partial_solutions(outgoing_node_pairs_set: NodePairSet, exclusion_list: list[set],
#                           outgoing_node_directions_dict: FittingDirections, state_grid: StateGrid, part_stock: PartStock,
#                           path_problem: PathProblem,
#                           ) -> list[Solution]:
#     """
#     Tries to generate partial solutions with the given node pairs. Node pairs may be excluded if
#     no solution to any other node is found. Node pairs in the same tuple are prevented from connecting.
#
#     Args:
#
#         outgoing_node_pairs_set (:obj:`~type_aliases.NodePairset`): Set of node pairs that represent the end points of a layout
#         exclusion_list (:obj:`list` [:obj:`set`]): List of node pairs that are excluded from connecting.
#         outgoing_node_directions_dict (:obj:`~type_aliases.FittingDirections`): Dictionary containing the directions each node can be connected to.
#         state_grid(:obj:`~type_aliases.StateGrid`): See :obj:`~type_aliases.StateGrid`
#         part_stock(:obj:`~type_aliases.PartStock`): See :paramref:`~process_state.ProcessState.part_stock`.
#         path_problem(:class:`PathProblem<path_problem>`): See :class:'PathProblem'.
#
#     """
#
#     start = path_problem.start_pos
#
#     partial_solutions = []
#
#     all_points_dict = {}  # reference of outgoing position to its connection
#
#     # get dictionary of all outgoing positions referencing their connection
#     for end_points in outgoing_node_pairs_set:
#         for point_tuple in end_points:
#             if point_tuple == ():
#                 continue
#             all_points_dict[point_tuple] = end_points
#
#     # sort all
#     all_points_list = sorted(all_points_dict.keys(), key=lambda x: manhattan_distance(start, x))
#     neighbors = deepcopy(all_points_list)
#     # todo: use a* algorithm
#     solution_dict = {}
#
#     while all_points_list:
#         solution_list = []
#         point_pos = all_points_list.pop(0)
#         point_outgoing_node_pairs_ref = all_points_dict[point_pos]
#         solution = None
#         for neighbor in neighbors:
#             if neighbor in point_outgoing_node_pairs_ref:
#                 continue
#             if {point_pos, neighbor} in exclusion_list:
#                 continue
#
#             partial_path_problem = PathProblem(algorithm=path_problem.algorithm, weights=path_problem.weights,
#                                                start_pos=point_pos,
#                                                start_directions=outgoing_node_directions_dict[point_pos],
#                                                goal_pos=neighbor,
#                                                goal_directions=outgoing_node_directions_dict[neighbor],
#                                                part_cost=path_problem.part_cost, part_stock=part_stock,
#                                                starting_part=0,
#                                                state_grid=deepcopy(state_grid),
#                                                transition_points=path_problem.transition_points)
#
#             draw_debug = False
#             solution = get_solution(path_problem=partial_path_problem, draw_debug=draw_debug)
#
#             if not solution:
#                 # try again, but exclude this connection combination
#                 exclusion_list.append({point_pos, neighbor})
#                 continue
#
#             heapq.heappush(solution_list, (solution.score, neighbor, solution))
#
#         # all_points_list.remove(point_pos)
#         neighbors.remove(point_pos)
#         # get connecting node with best score, then remove it
#         if solution_list:
#             best = heapq.heappop(solution_list)
#             best_point = best[1]
#             best_solution = best[2]
#             exclusion_list.append({point_pos, best_point})
#             all_points_list.remove(best_point)
#             neighbors.remove(best_point)
#
#             partial_solutions.append(best_solution)
#             part_stock = solution.part_stock  # adjust part stock for next iteration
#             # state_grid = solution.state_grid
#
#     return partial_solutions


def fuse_partial_solutions(partial_solutions: list[Solution], completed_layouts: BuildingInstructions,
                           initial_path_problem: PathProblem) -> Solution:
    """Fuses partial solutions and completed layouts into a complete solution from start to goal of the initial path problem.
    Only partially checks the validity of the complete solution. Partial solutions and completed layouts must therefore
    be compatible and equal to a valid assembly layout.

    Args:
        partial_solutions (:obj:`list` [:class:`Solution<solution>`]): A list of solutions that (presumably) form a complete solution.
        completed_layouts (:obj:`~class_types.BuildingInstructions`): Dictionary containing only layouts that are fully completed.
        initial_path_problem (:class:`PathProblem<path_problem>`): The original path problem.

    Returns:
        :class:`Solution<solution>` that is a fusion of all partial solutions.
    """
    node_trail = {}

    unordered_layouts = set()
    rendering_dict = {}

    # add solution information from already completed layouts
    for layout_trail, building_instruction in completed_layouts.items():
        building_instruction: BuildingInstruction
        layout_trail: Trail

        unordered_layouts.add(layout_trail)  # add for use later

        fit_first = building_instruction.required_fit_positions[0]
        fit_last = building_instruction.required_fit_positions[1]

        rendering_dict[fit_first] = const.fitting_id
        rendering_dict[layout_trail[1]] = building_instruction.pipe_id
        rendering_dict[fit_last] = const.fitting_id

        for pos in layout_trail:
            if pos in building_instruction.required_fit_positions:
                node_trail[pos] = const.fitting_id
            else:
                node_trail[pos] = building_instruction.pipe_id

    # also update total definite trail from partial solutions
    for partial_solution in partial_solutions:
        node_trail.update(partial_solution.node_trail)

    # get information from last partial solution iteration
    last_partial_solution = partial_solutions[0]
    part_stock = last_partial_solution.part_stock
    algorithm = last_partial_solution.algorithm
    state_grid = last_partial_solution.state_grid

    # add partial solution layout_trails to unordered layouts, too
    for partial_solution in partial_solutions:
        for layout_trail in partial_solution.ordered_trails:
            unordered_layouts.add(layout_trail)

    # sort unordered layouts
    ordered_layouts = []

    # search for the first layout
    add_layout = ()
    for layout_trail in unordered_layouts:
        if initial_path_problem.start_pos in layout_trail:
            add_layout = layout_trail
            break

    if add_layout[-1] == initial_path_problem.start_pos:
        unordered_layouts.remove(add_layout)
        add_layout = add_layout[::-1]
        unordered_layouts.add(add_layout)

    while add_layout in unordered_layouts:
        unordered_layouts.remove(add_layout)
        ordered_layouts.append(add_layout)
        for other_layout in unordered_layouts:
            if add_layout[-1] in (other_layout[0], other_layout[-1]):
                add_layout = other_layout
                break

    # todo: if needed, recalculate score
    score = last_partial_solution.score
    fused_solution: Solution = Solution(part_stock=part_stock, path_problem=initial_path_problem,
                                        rendering_dict=rendering_dict,
                                        algorithm=algorithm, state_grid=state_grid, score=score,
                                        ordered_trails=tuple(ordered_layouts),
                                        node_trail=node_trail)
    return fused_solution
