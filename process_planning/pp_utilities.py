import event_interpreting
from data_class.BuildingInstruction import BuildingInstruction
from data_class.Solution import Solution
from process_planning.ProcessState import ProcessState
from path_finding.common_types import Trail
from path_finding.path_math import get_direction, diff_pos, manhattan_distance
from constants import horizontal_directions, vertical_directions
from process_planning.ps_util import get_optimal_attachment_pos


def determine_next_part(layout_state:BuildingInstruction):
    next_part_id = None

    if not layout_state.att_set:
        # offer attachment
        next_part_id = -1
    elif not layout_state.pipe_set:
        next_part_id = layout_state.pipe_id
    elif len(layout_state.fit_set) < 2:
        next_part_id = 0

    return next_part_id


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

def deviated_from_path(current_state: ProcessState, optimal_solution: Solution):

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

        construction_layout[tuple(layout_trail)] = BuildingInstruction(att_set=set(), pipe_set=set(),
                                                                       fit_set=add_fit, pipe_id=pipe_id,
                                                                       required_fit_positions=(layout_trail[0],layout_trail[-1]),
                                                                       recommended_attachment_pos=rec_att_pos)


    return construction_layout

def prepare_initial_state(solution:Solution) -> ProcessState:
    construction_layouts = get_initial_construction_layouts(solution)

    state = ProcessState(state_grid=solution.path_problem.state_grid, part_stock=solution.path_problem.part_stock,
                         aimed_solution=solution, latest_layout=solution.layouts[0]  # starting with first layout
                         , construction_layouts=construction_layouts)
    return state

def get_total_definite_trail_from_construction_layouts(construction_layouts: dict[Trail:BuildingInstruction]) -> dict:
    total_definite_trail = {}
    for trail in construction_layouts.keys():
        layout_state = construction_layouts[trail]
        if layout_state.completed:
            for pos in layout_state.fit_set:
                total_definite_trail[pos] = 0

            for idx, pos in enumerate(trail, start=1):
                if idx >= len(trail)-1:
                    break
                total_definite_trail[pos] = layout_state.part_id

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
    """Returns all outgoing points in a layout as a connection. Interpolates layouts that are connected."""

    outgoing_connections_set = set()
    for layout_state in layouts.values():
        outgoing_connections_set.add(layout_state.required_fit_positions)

    layout_state_list = [i for i in layouts.values()]

    layout_state = layout_state_list.pop(0)

    while layout_state_list:
        for other_layout_state in layout_state_list:
            fit_positions = layout_state.required_fit_positions
            other_fit_positions = other_layout_state.required_fit_positions
            fit_positions_set = set(fit_positions)
            other_fit_positions_set = set(other_fit_positions)
            intersection = fit_positions_set.intersection(other_fit_positions_set)
            if intersection:
                outgoing_connections_set.discard(fit_positions)
                outgoing_connections_set.discard(other_fit_positions)

                intersected_pos = intersection.pop()
                fit_positions_set.discard(intersected_pos)
                other_fit_positions_set.discard(intersected_pos)

                fits_left = (fit_positions_set.pop(), other_fit_positions_set.pop())
                new_end_points = (fits_left[0], fits_left[1])

                outgoing_connections_set.add(new_end_points)
        layout_state = layout_state_list.pop()

    return outgoing_connections_set


def get_outgoing_directions(layouts):
    """returns the connecting directions the fittings of each layout requires"""

    direction_dict = {}

    for layout_trail in layouts.keys():
        fit_pos = layouts[layout_trail].required_fit_positions
        direction = get_direction(diff_pos(fit_pos[0], fit_pos[1]))
        if direction in horizontal_directions:
            direction_dict[fit_pos[0]] = direction_dict[fit_pos[1]] = vertical_directions

        elif direction in vertical_directions:
            direction_dict[fit_pos[0]] = direction_dict[fit_pos[1]] = horizontal_directions

    return direction_dict

