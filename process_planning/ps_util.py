import numpy as np

import path_finding.restrictions
from type_dictionary.common_types import Pos, Trail, Layouts

from data_class.BuildingInstruction import BuildingInstruction
from path_finding.path_math import get_direction, diff_pos, manhattan_distance


def get_optimal_attachment_pos(state_grid: np.ndarray, pos: Pos, part_id: int, direction: Pos):
    # todo: refactor
    countList = []
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
        if not path_finding.restrictions.out_of_bounds(b_left,
                                                                state_grid) and not path_finding.restrictions.out_of_bounds(
                b_right, state_grid):
            if state_grid[b_left] != 0 or state_grid[b_right] != 0:
                continue
            else:
                rec_att_pos = (pos[0] + i * direction[0] + 1, pos[1] + i * direction[1] + 1)
                return rec_att_pos
        else:
            rec_att_pos = (pos[0] + i * direction[0] + 1, pos[1] + i * direction[1] + 1)
            return rec_att_pos
    else:  # second best option: one side is empty
        for i in sorter:  # check pos right of pipe
            if i not in countList:
                continue
            n_left = (left[0] + (direction[0] * (i)), left[1] + (direction[1] * (i)))
            b_left = (pos[0] + n_left[0], pos[1] + n_left[1])
            n_right = (right[0] + (direction[0] * (i)), right[1] + (direction[1] * (i)))
            b_right = (pos[0] + n_right[0], pos[1] + n_right[1])
            if not path_finding.restrictions.out_of_bounds(b_left,
                                                                    state_grid) and not path_finding.restrictions.out_of_bounds(
                    b_right, state_grid):
                if state_grid[b_left] != 0 and state_grid[b_right] != 0:
                    continue
                else:
                    rec_att_pos = (pos[0] + i * direction[0] + 1, pos[1] + i * direction[1] + 1)
                    break
            else:
                rec_att_pos = (pos[0] + i * direction[0] + 1, pos[1] + i * direction[1] + 1)
                break
        else:
            rec_att_pos = pos
        return rec_att_pos


def get_neighboring_layouts(current_layout: Trail, layouts: Layouts) -> list[Trail]:
    neighboring_layouts = []
    idx = layouts.index(current_layout)

    if idx + 1 < len(layouts):
        neighboring_layouts.append(layouts[idx + 1])
    if idx - 1 >= 0:
        neighboring_layouts.append(layouts[idx - 1])

    return neighboring_layouts


def get_detour_trail(length: int, direction: Pos, fit_pos: tuple) -> Trail:
    # create new state

    # if any of the following pos is None, there was an error
    pipe_start_pos = None
    pipe_end_pos = None

    # update state grid
    construction_trail = []

    # set construction_parts fittings
    construction_parts = {fit_pos[0]: 0, fit_pos[1]: 0}

    for i in range(length + 1):
        state_pos = (fit_pos[0][0] + direction[0] * i, fit_pos[0][1] + direction[1] * i)
        construction_trail.append(state_pos)

    return tuple(construction_trail)


def get_detour_state(length: int, att_set: set, pipe_set: set, fit_tup: tuple, state_grid: np.ndarray,
                     possible_att_pipe_positions: Trail):
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


def get_building_instructions_from_solution(solution):
    construction_layout = {}
    start = solution.path_problem.start_pos
    goal = solution.path_problem.goal_pos

    for layout_trail in solution.ordered_trails:
        add_fit = set()
        pipe_id = solution.absolute_trail[layout_trail[1]]
        rec_att_pos = get_optimal_attachment_pos(state_grid=solution.path_problem.state_grid,
                                                 direction=get_direction(diff_pos(layout_trail[0], layout_trail[1])),
                                                 part_id=manhattan_distance(layout_trail[1], layout_trail[-1]),
                                                 pos=layout_trail[1])
        if layout_trail[0] == start:
            add_fit.add(start)
        elif layout_trail[-1] == goal:
            add_fit.add(goal)

        possible_att_pipe_positions = [i for i in layout_trail if i != layout_trail[0] and i != layout_trail[-1]]

        construction_layout[tuple(layout_trail)] = BuildingInstruction(pipe_id=pipe_id,
                                                                       required_fit_positions=(
                                                                           layout_trail[0], layout_trail[-1]),
                                                                       recommended_attachment_pos=rec_att_pos,
                                                                       possible_att_pipe_positions=tuple(
                                                                           possible_att_pipe_positions),)

    return construction_layout