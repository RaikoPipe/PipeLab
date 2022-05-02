from __future__ import annotations

import heapq
from copy import deepcopy

import numpy as np

from type_dictionary import constants as const
from ProcessPlanning.pp_data_class.BuildingInstruction import BuildingInstruction
from PathFinding.pf_data_class.PathProblem import PathProblem
from PathFinding.pf_data_class.Solution import Solution
from PathFinding.util.path_math import manhattan_distance
from PathFinding.solution_manager import get_solution
from type_dictionary.common_types import Trail


def get_partial_solutions(outgoing_node_pairs_set: set, closed_list: list[set],
                          outgoing_node_directions_dict: dict, state_grid: np.ndarray, part_stock: dict,
                          path_problem: PathProblem,
                          ) -> list[Solution]:
    """
    Tries to generate partial solutions with the given node pairs. Node pairs may be excluded if
    no solution to any other node is found. Node pairs in the same tuple are prevented from connecting.

    Args:

        outgoing_node_pairs_set: Set of node pairs that represent the end points of a layout
        closed_list: List of node pairs that are excluded from connecting.
        outgoing_node_directions_dict: Dictionary containing the directions each node can be connected to.
        state_grid: #todo: reference
        part_stock: todo: reference
        path_problem: :class: 'classes.PathProblem.PathProblem'

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
    neighbors = deepcopy(all_points_list)
    # todo: use a* algorithm
    solution_dict = {}

    while all_points_list:
        solution_list = []
        point_pos = all_points_list.pop(0)
        point_outgoing_node_pairs_ref = all_points_dict[point_pos]
        solution = None
        for neighbor in neighbors:
            if neighbor in point_outgoing_node_pairs_ref:
                continue
            if {point_pos, neighbor} in closed_list:
                continue

            partial_path_problem = PathProblem(algorithm=path_problem.algorithm, weights=path_problem.weights,
                                               start_pos=point_pos,
                                               start_directions=outgoing_node_directions_dict[point_pos],
                                               goal_pos=neighbor,
                                               goal_directions=outgoing_node_directions_dict[neighbor],
                                               part_cost=path_problem.part_cost, part_stock=part_stock,
                                               starting_part=0,
                                               state_grid=deepcopy(state_grid),
                                               transition_points=path_problem.transition_points)

            draw_debug = False
            solution = get_solution(path_problem=partial_path_problem, draw_debug=draw_debug)

            if not solution:
                # try again, but exclude this connection combination
                closed_list.append({point_pos, neighbor})
                continue

            heapq.heappush(solution_list, (solution.score, neighbor, solution))

        # all_points_list.remove(point_pos)
        neighbors.remove(point_pos)
        # get connecting node with best score, then remove it
        if solution_list:
            best = heapq.heappop(solution_list)
            best_point = best[1]
            best_solution = best[2]
            closed_list.append({point_pos, best_point})
            all_points_list.remove(best_point)
            neighbors.remove(best_point)

            partial_solutions.append(best_solution)
            part_stock = solution.part_stock  # adjust part stock for next iteration
            # state_grid = solution.state_grid

    return partial_solutions


def fuse_partial_solutions(partial_solutions: list[Solution], completed_layouts: dict[Trail,BuildingInstruction],
                           initial_path_problem: PathProblem):
    """Fuses partial solutions and completed layouts into a complete solution from start to goal of the initial path problem.
    Only partially checks the validity of the complete solution. Partial solutions and completed layouts must therefore
    be compatible and equal to a valid assembly layout."""
    absolute_trail = {}

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
                absolute_trail[pos] = const.fitting_id
            else:
                absolute_trail[pos] = building_instruction.pipe_id

    # also update total definite trail from partial solutions
    for partial_solution in partial_solutions:
        absolute_trail.update(partial_solution.absolute_trail)

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
    fused_solution = Solution(part_stock=part_stock, path_problem=initial_path_problem, rendering_dict=rendering_dict,
                              algorithm=algorithm, state_grid=state_grid, score=score, ordered_trails=ordered_layouts,
                              absolute_trail=absolute_trail)
    return fused_solution