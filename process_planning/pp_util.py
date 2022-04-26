from copy import deepcopy
from typing import Optional

import constants
from constants import horizontal_directions, vertical_directions
from data_class.BuildingInstruction import BuildingInstruction
from data_class.Solution import Solution
from path_finding import partial_solver
from type_dictionary.common_types import Trail, Pos
from path_finding.path_math import get_direction, diff_pos, manhattan_distance
from process_planning.ProcessState import ProcessState
from process_planning.ps_util import get_optimal_attachment_pos

message_dict = {1: "fitting", 2: "pipe", 3: "attachment"}

def determine_next_part(process_state:ProcessState, layout:Trail):
    next_part_id = None

    building_instruction = process_state.building_instructions.get(layout)
    completed, build_progress = process_state.completed_instruction(building_instruction)

    if not building_instruction:
        return None

    if not completed:
        if build_progress == 2:
            next_part_id = building_instruction.pipe_id
        # elif build_progress == 1:
        #     next_part_id = constants.fitting_id

    return next_part_id

def get_absolute_trail_from_building_instructions(building_instructions: dict[Trail:BuildingInstruction]) -> dict[Pos:int]:
    absolute_trail = {}
    for trail in building_instructions.keys():
        layout_state = building_instructions[trail]
        if layout_state.layout_completed:
            for pos in layout_state.fit_set:
                absolute_trail[pos] = 0

            for idx, pos in enumerate(trail, start=1):
                if idx >= len(trail) - 1:
                    break
                absolute_trail[pos] = layout_state.part_id

    return absolute_trail


def get_completed_instructions(building_instructions) -> dict[Trail:BuildingInstruction]:
    """Returns all completed instructions."""
    completed_instructions = {}
    for layout_trail in building_instructions.keys():
        instruction = building_instructions[layout_trail]
        if instruction.layout_completed:
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


def adjust_pos_in_node_pairs_set(node_pairs_set, pos):
    """remove pos from first connection that contains it. If not found, then add as a connection containing only pos."""
    for connection in node_pairs_set:
        if pos in connection:
            set_connection = set(connection)
            set_connection.discard(pos)
            node_pairs_set.discard(connection)
            node_pairs_set.add((set_connection.pop(), ()))
            break
    else:
        node_pairs_set.add((pos, ()))


def get_solution_on_detour_event(initial_path_problem, process_state, detour_event) -> Optional[Solution]:
    """Tries to get a new solution on a detour event."""
    completed_instructions = get_completed_instructions(process_state.building_instructions)

    path_problem = deepcopy(initial_path_problem)

    # get state grid of completed layouts
    current_state_grid = path_problem.state_grid

    for layout_state in completed_instructions.keys():
        for pos in layout_state:
            current_state_grid[pos] = 2

    # get part stock (assuming only completed layouts)
    current_part_stock = path_problem.part_stock

    for layout_state in completed_instructions.values():
        current_part_stock[0] -= len(layout_state.required_fit_positions)
        current_part_stock[layout_state.pipe_id] -= 1

    layout_outgoing_node_pairs_set = get_outgoing_node_pairs(completed_instructions)
    layout_outgoing_directions_dict = get_outgoing_node_directions(completed_instructions)

    start = path_problem.start_pos
    goal = path_problem.goal_pos

    # Add start/goal to the node_pairs set. If already in a connection, discard.
    adjust_pos_in_node_pairs_set(layout_outgoing_node_pairs_set, start)
    adjust_pos_in_node_pairs_set(layout_outgoing_node_pairs_set, goal)

    layout_outgoing_directions_dict[
        initial_path_problem.start_pos] = initial_path_problem.start_directions
    layout_outgoing_directions_dict[
        initial_path_problem.goal_pos] = initial_path_problem.goal_directions

    exclusion_list = [{start, goal}]  # contains node_pairs that are excluded from partial solving

    # get new solution
    partial_solutions = partial_solver.get_partial_solutions(
        outgoing_node_pairs_set=layout_outgoing_node_pairs_set,
        outgoing_node_directions_dict=layout_outgoing_directions_dict,
        exclusion_list=exclusion_list,
        part_stock=current_part_stock,
        state_grid=current_state_grid,
        path_problem=path_problem)
    solution = None

    detour_trail = list(detour_event)[0][0]

    for partial_solution in partial_solutions:
        for trail in partial_solution.ordered_trails:

            if detour_trail in trail:
                solution = partial_solver.fuse_partial_solutions(partial_solutions, completed_instructions,
                                                                 initial_path_problem)
                break

    return solution


def make_registration_message(event_pos: Pos, event_code: int, removal: bool, pipe_id: int) -> str:
    """Returns a message as string, confirming a placement or removal event."""
    object_name = message_dict[event_code]
    motion_type = "placement"
    if removal:
        motion_type = "removal"

    if pipe_id not in (0, -1, None):
        message = str.format(
            f"Process Planner: Registered {motion_type} for object {object_name} (ID {pipe_id}) at Position {event_pos}")
    elif pipe_id == -2:
        message = str.format(
        f"Process Planner: Registered {motion_type} for object {object_name} (ID Unknown) at Position {event_pos}")
    else:
        message = str.format(
            f"Process Planner: Registered {motion_type} for object {object_name} at Position {event_pos}")

    return message


def make_special_message(message: str, event_pos: Pos):
    """Returns a special message as string, usually used in case of deviated placements."""
    message = str.format(f"Process Planner: Position {event_pos}: {message} ")
    return message


def make_error_message(event_pos: Pos, additional_message: str):
    """Returns error messages as string containing the position where the error occurred as well as additional
    information regarding the reason for the error."""
    message = str.format(f"Process Planner: Process error at Position {event_pos}: {additional_message}")
    return message



