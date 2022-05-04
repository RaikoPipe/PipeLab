from __future__ import annotations

import PathFinding.path_finding_util.restrictions
from PathFinding.path_finding_util.path_math import get_direction, diff_pos, manhattan_distance
from PathFinding.pf_data_class.Solution import Solution
from type_dictionary.special_types import *


def get_optimal_attachment_pos(state_grid: StateGrid, pos: Pos, part_id: int, direction: Pos) -> Pos:
    """Returns the best attachment position. This is evaluated by two factors: The amount of obstacles surrounding the position
     and the distance relative to the middle of the part.

     :param part_id: Part ID
     :param direction: The direction though which positions are iterated through the function
     :param pos: Starting position of a pipe.
     :param state_grid: See type for explanation. Only parts and obstacles are considered as obstacles.
     :return: rec_att_pos: Optimal attachment position"""
    # old function that needs refactoring, but fulfills its purpose nonetheless
    countList = []
    considered_obstacles = {1, 2}
    sorter = [int(part_id / 2), 3, 1, 2, 4, 0, 5, 6]
    for i in range(0, part_id):
        countList.append(i)
    left = (-direction[1], -direction[0])
    right = (direction[1], direction[0])
    # first best option: both sides are empty
    for i in sorter:  # check pos right of pipe
        if i not in countList:
            continue
        n_left = (left[0] + (direction[0] * (i)), left[1] + (direction[1] * (i)))
        b_left = (pos[0] + n_left[0], pos[1] + n_left[1])
        n_right = (right[0] + (direction[0] * (i)), right[1] + (direction[1] * (i)))
        b_right = (pos[0] + n_right[0], pos[1] + n_right[1])
        if not PathFinding.path_finding_util.restrictions.out_of_bounds(b_left,
                                                                        state_grid) and not PathFinding.path_finding_util.restrictions.out_of_bounds(
            b_right, state_grid):
            if state_grid[b_left] in considered_obstacles or state_grid[b_right] in considered_obstacles:
                continue
            else:
                rec_att_pos = (pos[0] + i * direction[0], pos[1] + i * direction[1])
                return rec_att_pos
        else:
            rec_att_pos = (pos[0] + i * direction[0], pos[1] + i * direction[1])
            return rec_att_pos
    else:  # second best option: one side is empty
        for i in sorter:  # check pos right of pipe
            if i not in countList:
                continue
            n_left = (left[0] + (direction[0] * (i)), left[1] + (direction[1] * (i)))
            b_left = (pos[0] + n_left[0], pos[1] + n_left[1])
            n_right = (right[0] + (direction[0] * (i)), right[1] + (direction[1] * (i)))
            b_right = (pos[0] + n_right[0], pos[1] + n_right[1])
            if not PathFinding.path_finding_util.restrictions.out_of_bounds(b_left,
                                                                            state_grid) and not PathFinding.path_finding_util.restrictions.out_of_bounds(
                b_right, state_grid):
                if state_grid[b_left] in considered_obstacles and state_grid[b_right] in considered_obstacles:
                    continue
                else:
                    rec_att_pos = (pos[0] + i * direction[0], pos[1] + i * direction[1])
                    break
            else:
                rec_att_pos = (pos[0] + i * direction[0], pos[1] + i * direction[1])
                break
        else:
            rec_att_pos = pos
        return rec_att_pos


def get_neighboring_layouts(trail: Trail, ordered_trails: OrderedTrails) -> TrailList:
    """Returns all trails that are neighbors to current_layout according to ordered_trails.

    :param trail: The layout to which neighbors are to be found
    :param ordered_trails: See ordered_trails in :attr:`~Solution.ordered_trails`
    :return: List containing neighboring trails"""
    neighboring_layouts = []
    idx = ordered_trails.index(trail)

    if idx + 1 < len(ordered_trails):
        neighboring_layouts.append(ordered_trails[idx + 1])
    if idx - 1 >= 0:
        neighboring_layouts.append(ordered_trails[idx - 1])

    return neighboring_layouts


def construct_trail(length: int, direction: Pos, pos: Pos) -> Trail:
    """Constructs and returns a trail by iterating through length starting from fit_pos towards direction.

    :param length: Length of the trail.
    :param direction: Direction from fit_pos.
    :param pos: Starting position."""
    # update state grid
    trail = []

    for i in range(length + 1):
        state_pos = (pos[0] + direction[0] * i, pos[1] + direction[1] * i)
        trail.append(state_pos)

    return tuple(trail)


def construct_detour_building_instruction(length: int, fit_tup: NodePair, state_grid: StateGrid,
                                          possible_att_pipe_positions: Trail) -> BuildingInstruction:
    """Constructs a building instruction from given parameters.

    :param length: Length of the trail.
    :param fit_tup: Tuple containing two positions
    :param state_grid: See type
    :param possible_att_pipe_positions: Trail containing positions the pipe of this new layout needs to occupy.
    """
    pipe_id = length - 1
    direction = get_direction(diff_pos(fit_tup[0], fit_tup[1]))
    first_pipe_pos = (fit_tup[0][0] * direction[0], fit_tup[0][1] * direction[1])

    rec_att_pos = get_optimal_attachment_pos(state_grid=state_grid,
                                             direction=direction,
                                             part_id=pipe_id,
                                             pos=first_pipe_pos)

    layout_state = BuildingInstruction(pipe_id=pipe_id, required_fit_positions=(fit_tup[0], fit_tup[1]),
                                       recommended_attachment_pos=rec_att_pos,
                                       layout_completed=True, possible_att_pipe_positions=possible_att_pipe_positions)

    return layout_state


def construct_building_instructions_from_solution(solution: Solution) -> BuildingInstructions:
    """Constructs building instructions from the given solution.

    :param solution: See :class:`~Solution`
    :return: See :attr:`ProcessState.building_instructions`"""
    building_instructions = {}
    start = solution.path_problem.start_pos
    goal = solution.path_problem.goal_pos

    for layout_trail in solution.ordered_trails:
        add_fit = set()
        pipe_id = solution.node_trail[layout_trail[1]]
        rec_att_pos = get_optimal_attachment_pos(state_grid=solution.path_problem.state_grid,
                                                 direction=get_direction(diff_pos(layout_trail[0], layout_trail[1])),
                                                 part_id=manhattan_distance(layout_trail[1], layout_trail[-1]),
                                                 pos=layout_trail[1])
        if layout_trail[0] == start:
            add_fit.add(start)
        elif layout_trail[-1] == goal:
            add_fit.add(goal)

        possible_att_pipe_positions = [i for i in layout_trail if i != layout_trail[0] and i != layout_trail[-1]]

        building_instructions[tuple(layout_trail)] = BuildingInstruction(pipe_id=pipe_id,
                                                                         required_fit_positions=(
                                                                             layout_trail[0], layout_trail[-1]),
                                                                         recommended_attachment_pos=rec_att_pos,
                                                                         possible_att_pipe_positions=tuple(
                                                                             possible_att_pipe_positions), )

    return building_instructions


def get_completion_proportion(building_instructions: BuildingInstructions) -> float:
    """Calculates the proportion of building instructions with a true state in the attribute layout_completed.

    :param building_instructions: See :attr:`ProcessState.building_instructions`
    :return: Completion Proportion"""
    count = 0
    for layout, instruction in building_instructions.items():
        if instruction.layout_completed:
            count += 1
    proportion = count / len(building_instructions.keys())
    return proportion
