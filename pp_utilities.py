import numpy as np

import event_interpreting
import path_finding.path_math
import path_finding.restriction_functions
from data_class.LayoutState import LayoutState
from data_class.Solution import Solution
from data_class.ConstructionState import State
from path_finding.common_types import Pos, Trail, DefinitePath
from path_finding.path_math import get_direction, diff_pos, manhattan_distance

def determine_next_part(layout_state:LayoutState):
    next_part_id = None

    if not layout_state.att_set:
        # offer attachment
        next_part_id = -1
    elif not layout_state.pipe_set:
        next_part_id = layout_state.pipe_id
    elif len(layout_state.fit_set) < 2:
        next_part_id = 0

    return next_part_id

def get_deviation_trail(length:int, direction:Pos, fit_pos:tuple) -> Trail:
    # create new state

    # if any of the following pos is None, there was an error
    pipe_start_pos = None
    pipe_end_pos = None

    # update state grid
    construction_trail = []

    #set construction_parts fittings
    construction_parts = {fit_pos[0]: 0, fit_pos[1]: 0}

    for i in range(length + 1):
        state_pos = (fit_pos[0][0] + direction[0] * i, fit_pos[0][1] + direction[1] * i)
        construction_trail.append(state_pos)

    return tuple(construction_trail)


def get_deviation_state(length:int, att_set:set, pipe_set:set, fit_tup:tuple, state_grid:np.ndarray):
    pipe_id = length - 1
    direction = get_direction(diff_pos(fit_tup[0], fit_tup[1]))
    first_pipe_pos = (fit_tup[0][0] * direction[0], fit_tup[0][1] * direction[1])

    rec_att_pos = get_optimal_attachment_pos(state_grid=state_grid,
                                             direction=direction,
                                             part_id=pipe_id,
                                             pos=first_pipe_pos)

    layout_state = LayoutState(att_set=att_set, pipe_set=pipe_set, fit_set={fit_tup[0], fit_tup[1]},
                               pipe_id=pipe_id, required_fit_positions=(fit_tup[0], fit_tup[1]), recommended_attachment_pos=rec_att_pos,
                               completed = True)

    return layout_state


def get_optimal_attachment_pos(state_grid: np.ndarray, pos: Pos, part_id:int, direction:Pos):
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
        if not path_finding.restriction_functions.out_of_bounds(b_left, state_grid) and not path_finding.restriction_functions.out_of_bounds(b_right, state_grid):
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
            if not path_finding.restriction_functions.out_of_bounds(b_left, state_grid) and not path_finding.restriction_functions.out_of_bounds(b_right, state_grid):
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

# unused

def get_current_path(state_grid, part_stock):
    """Returns the current paths from a state_grid"""
    trail_list = event_interpreting.get_trails_from_state_grid(state_grid=state_grid, searched_state=2)
    path_list = []
    for trail in trail_list:
        path = event_interpreting.get_path_from_trail(trail)
        path, _ = event_interpreting.correct_path_start(path, part_stock)
        path_list.append(path)
    return path_list

def get_updated_motion_dict(new_pos, motion_dict):
    for pos, event in new_pos.items():
        if pos == 0 and pos in motion_dict:
            motion_dict.pop(pos)
        else:
            motion_dict[pos] = event

    return motion_dict

def deviated_from_path(current_state: State, optimal_solution: Solution):

    for connection in current_state.fc_set:
        if connection not in optimal_solution.fc_set:
            return True
    else:
        return False

    #todo: finish functions below

def get_initial_construction_layouts(solution):
    construction_layout = {}
    start = solution.path_problem.start_pos
    goal = solution.path_problem.goal_pos


    for layout_trail in solution.layouts:
        add_fit = set()
        pipe_id = solution.total_definite_trail[layout_trail[1]]
        rec_att_pos = get_optimal_attachment_pos(state_grid=solution.path_problem.state_grid,
                                                 direction=get_direction(diff_pos(layout_trail[0],layout_trail[1])),
                                                 part_id=manhattan_distance(layout_trail[1], layout_trail[-1]),
                                                 pos=layout_trail[1])
        if layout_trail[0] == start:
            add_fit.add(start)
        elif layout_trail[-1] == goal:
            add_fit.add(goal)

        construction_layout[tuple(layout_trail)] = LayoutState(att_set=set(),pipe_set=set(),
                                                            fit_set=add_fit, pipe_id=pipe_id,
                                                            required_fit_positions=(layout_trail[0],layout_trail[-1]),
                                                            recommended_attachment_pos=rec_att_pos)


    return construction_layout

def prepare_initial_state(solution:Solution) -> State:
    construction_layouts = get_initial_construction_layouts(solution)

    state = State(state_grid=solution.path_problem.state_grid,part_stock=solution.path_problem.part_stock,
                  aimed_solution=solution,latest_layout=solution.layouts[0] # starting with first layout
                  ,construction_layouts=construction_layouts)
    return state

def get_total_definite_trail_from_construction_layouts(construction_layouts: dict[Trail:LayoutState]) -> dict:
    total_definite_trail = {}
    for trail in construction_layouts.keys():
        layout_state = construction_layouts[trail]
        if layout_state.completed:
            for pos in layout_state.fit_set:
                total_definite_trail[pos] = 0

            for idx, pos in enumerate(trail, start=1):
                if idx >= len(trail)-1:
                    break
                total_definite_trail[pos] = layout_state.pipe_id

    return total_definite_trail


def get_completed_layouts(construction_layouts):
    """returns all completed layouts"""
    completed_layouts = {}
    for layout_trail in construction_layouts.keys():
        layout_state = construction_layouts[layout_trail]
        if layout_state.completed:
            completed_layouts[layout_trail] = layout_state

    return completed_layouts

def get_outgoing_connections(layouts):
    # todo: check if two completed layouts are connected
    outgoing_connections_set = set()
    for layout_state in layouts.items():
        outgoing_connections_set.add(layout_state.required_fit_positions)

    layout_state_list = list(layouts.items())

    layout_state = layout_state_list.pop(0)

    while layouts:
        for other_layout_state in layouts.items():
            fit_positions = set(layout_state.required_fit_positions)
            other_fit_positions = set(other_layout_state.required_fit_positions)
            intersection = fit_positions.intersection(other_fit_positions)
            if intersection:
                outgoing_connections_set.discard(layout_state.required_fit_positions)
                outgoing_connections_set.discard(other_layout_state.required_fit_positions)

                fit_positions.discard(intersection)
                other_fit_positions.discard(intersection)
                new_end_points = (fit_positions.pop, other_fit_positions.pop)

                outgoing_connections_set.add(new_end_points)
        layout_state = layout_state_list.pop()

    return outgoing_connections_set


def get_outgoing_directions(layouts):
    """returns the connecting directions the fittings of each layout requires"""
    horizontal_directions = {(1,0),(-1,0)}
    vertical_directions = {(0,1),(-1,0)}

    direction_dict = {}

    for layout_trail in layouts.keys():
        fit_pos = layout_trail.required_fit_positions
        direction = get_direction(diff_pos(fit_pos[0], fit_pos[1]))
        if direction in horizontal_directions:
            direction_dict[fit_pos[0]] = direction_dict[fit_pos[1]] = vertical_directions

        elif direction_dict in vertical_directions:
            direction_dict[fit_pos[0]] = direction_dict[fit_pos[1]] = horizontal_directions

    return direction_dict

