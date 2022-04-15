import event_interpreting
from constants import horizontal_directions, vertical_directions
from data_class.BuildingInstruction import BuildingInstruction
from data_class.Solution import Solution
from path_finding.common_types import Trail
from path_finding.path_math import get_direction, diff_pos, manhattan_distance
from process_planning.ProcessState import ProcessState
from process_planning.ps_util import get_optimal_attachment_pos


def determine_next_part(layout_state: BuildingInstruction):
    next_part_id = None

    if not layout_state.att_set:
        # offer attachment
        next_part_id = -1
    elif not layout_state.pipe_set:
        next_part_id = layout_state.pipe_id
    elif len(layout_state.fit_set) < 2:
        next_part_id = 0

    return next_part_id

def get_building_instructions_from_solution(solution: Solution) -> dict[Trail:BuildingInstruction]:
    """returns a dictionary containing trails pointing to their building instruction."""
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

        construction_layout[tuple(layout_trail)] = BuildingInstruction(att_set=set(), pipe_set=set(),
                                                                       fit_set=add_fit, pipe_id=pipe_id,
                                                                       required_fit_positions=(
                                                                       layout_trail[0], layout_trail[-1]),
                                                                       recommended_attachment_pos=rec_att_pos)

    return construction_layout


def get_initial_process_state_from_solution(solution: Solution) -> ProcessState:
    """Creates a new process state from a solution."""
    construction_layouts = get_building_instructions_from_solution(solution)

    state = ProcessState(state_grid=solution.path_problem.state_grid, part_stock=solution.path_problem.part_stock,
                         aimed_solution=solution, latest_layout=solution.ordered_trails[0]  # starting with first layout
                         , construction_layouts=construction_layouts)
    return state


def get_absolute_trail_from_building_instructions(building_instructions: dict[Trail:BuildingInstruction]) -> dict:
    total_definite_trail = {}
    for trail in building_instructions.keys():
        layout_state = building_instructions[trail]
        if layout_state.completed:
            for pos in layout_state.fit_set:
                total_definite_trail[pos] = 0

            for idx, pos in enumerate(trail, start=1):
                if idx >= len(trail) - 1:
                    break
                total_definite_trail[pos] = layout_state.part_id

    return total_definite_trail


def get_completed_instructions(building_instructions) -> dict[Trail:BuildingInstruction]:
    """Returns all completed instructions."""
    completed_instructions = {}
    for layout_trail in building_instructions.keys():
        instruction = building_instructions[layout_trail]
        if instruction.completed:
            completed_instructions[layout_trail] = instruction

    return completed_instructions


def get_outgoing_node_pairs(building_instructions: dict[Trail:BuildingInstruction]):
    """Returns all outgoing points in building instructions as a connection. Interpolates them if they are
    connected."""

    outgoing_node_pairs_set = set()
    for layout_state in building_instructions.values():
        outgoing_node_pairs_set.add(layout_state.required_fit_positions)

    layout_state_list = [i for i in building_instructions.values()]

    layout_state = layout_state_list.pop(0)

    while layout_state_list:
        for other_layout_state in layout_state_list:
            fit_positions = layout_state.required_fit_positions
            other_fit_positions = other_layout_state.required_fit_positions
            fit_positions_set = set(fit_positions)
            other_fit_positions_set = set(other_fit_positions)
            intersection = fit_positions_set.intersection(other_fit_positions_set)
            if intersection:
                outgoing_node_pairs_set.discard(fit_positions)
                outgoing_node_pairs_set.discard(other_fit_positions)

                intersected_pos = intersection.pop()
                fit_positions_set.discard(intersected_pos)
                other_fit_positions_set.discard(intersected_pos)

                fits_left = (fit_positions_set.pop(), other_fit_positions_set.pop())
                new_end_points = (fits_left[0], fits_left[1])

                outgoing_node_pairs_set.add(new_end_points)
        layout_state = layout_state_list.pop()

    return outgoing_node_pairs_set


def get_outgoing_node_directions(building_instructions: dict[Trail:BuildingInstruction]):
    """Returns the connecting directions the fittings of each building instruction requires."""

    direction_dict = {}

    for layout_trail in building_instructions.keys():
        fit_pos = building_instructions[layout_trail].required_fit_positions
        direction = get_direction(diff_pos(fit_pos[0], fit_pos[1]))
        if direction in horizontal_directions:
            direction_dict[fit_pos[0]] = direction_dict[fit_pos[1]] = vertical_directions

        elif direction in vertical_directions:
            direction_dict[fit_pos[0]] = direction_dict[fit_pos[1]] = horizontal_directions

    return direction_dict
