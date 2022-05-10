from __future__ import annotations

from copy import deepcopy
from typing import Optional

from PathFinding import partial_solver
from PathFinding.path_finding_util.path_math import get_direction, diff_pos
from PathFinding.pf_data_class.PathProblem import PathProblem
from PathFinding.pf_data_class.Solution import Solution
from ProcessPlanning.ProcessState import ProcessState
from ProcessPlanning.pp_data_class.BuildingInstruction import BuildingInstruction
from ProcessPlanning.pp_data_class.PickEventInfo import PickEventInfo
from ProcessPlanning.pp_data_class.PlacementEventInfo import PlacementEventInfo
from TypeDictionary import constants
from TypeDictionary.class_types import *
from TypeDictionary.constants import horizontal_directions, vertical_directions

message_dict = {1: "fitting", 2: "pipe", 3: "attachment"}


def determine_next_part(process_state: ProcessState, layout: Trail) -> Optional[int]:
    """Determines which part should be picked next according to the current build progress.

    Args:
        process_state(:class:`ProcessState`): The current process state.
        layout(:obj:`~type_aliases.Trail`): The current layout.

    Returns:
        Part ID :obj:`int` of the determined part."""
    next_part_id = None

    building_instruction = process_state.building_instructions.get(layout)
    if not building_instruction:
        return None

    completed, build_progress = process_state.completed_instruction(building_instruction)

    if not completed:
        if build_progress == 2:
            next_part_id = building_instruction.pipe_id
        # elif build_progress == 1:
        #     next_part_id = constants.fitting_id

    return next_part_id


def get_completed_instructions(building_instructions: BuildingInstructions) -> \
        BuildingInstructions:
    """Looks for completed instructions inside building_instructions and returns them.

    Args:
        building_instructions(:obj:`~class_types.BuildingInstructions`): See :paramref:`~ProcessState.ProcessState.building_instructions`.

    Return:
        All completed instructions (:obj:`~class_types.BuildingInstructions`)

    """
    completed_instructions = {}
    for layout_trail in building_instructions.keys():
        instruction = building_instructions[layout_trail]
        if instruction.layout_completed:
            completed_instructions[layout_trail] = instruction

    return completed_instructions


def get_outgoing_node_pairs(building_instructions: BuildingInstructions) -> NodePairSet:
    """Returns all outgoing points in building_instructions as a connection. Interpolates them if they are
    connected.

    Args:
        building_instructions(:obj:`~class_types.BuildingInstructions`): See :paramref:`~ProcessState.ProcessState.building_instructions`.

    Returns:
        :obj:`~type_aliases.NodePairSet`

    """

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


def get_outgoing_node_directions(building_instructions: BuildingInstructions) -> FittingDirections:
    """Returns the connecting directions the fittings of each building instruction requires.

    Args:
        building_instructions(:obj:`~class_types.BuildingInstructions`): See :paramref:`~ProcessState.ProcessState.building_instructions`.

    Returns:
        :obj:`set` with all outgoing node directions.
    """

    direction_dict = {}

    for layout_trail in building_instructions.keys():
        fit_pos = building_instructions[layout_trail].required_fit_positions
        direction = get_direction(diff_pos(fit_pos[0], fit_pos[1]))
        if direction in horizontal_directions:
            direction_dict[fit_pos[0]] = direction_dict[fit_pos[1]] = vertical_directions

        elif direction in vertical_directions:
            direction_dict[fit_pos[0]] = direction_dict[fit_pos[1]] = horizontal_directions

    return direction_dict


def adjust_pos_in_node_pairs_set(node_pairs_set: NodePairSet, pos: Pos):
    """Remove pos from first node pair tuple in node_pairs_set that contains it. If not found,
    then add as a node pair tuple containing only pos.

    Args:
        pos(:obj:`~type_aliases.Pos`): Pos to be evaluated.
        node_pairs_set(:obj:`~type_aliases.NodePairSet`): Set with all outgoing node pairs."""
    for connection in node_pairs_set:
        if pos in connection:
            set_connection = set(connection)
            set_connection.discard(pos)
            node_pairs_set.discard(connection)
            node_pairs_set.add((set_connection.pop(), ()))
            break
    else:
        node_pairs_set.add((pos, ()))


def get_solution_on_detour_event(initial_path_problem: PathProblem, process_state: ProcessState, detour_event) -> \
        Optional[Solution]:
    """Tries to get a new solution on a detour event.
    Args:

    initial_path_problem (:class:`PathProblem`): The original path problem.
    process_state (:class:`ProcessState`): The current process state.

    Returns:
        :class:`Solution` if one was found, else None.
    """
    completed_instructions = get_completed_instructions(process_state.building_instructions)

    path_problem = deepcopy(initial_path_problem)

    # get state grid of completed layouts
    current_state_grid = path_problem.state_grid

    for layout_state in completed_instructions.keys():
        for pos in layout_state:
            current_state_grid[pos] = 2

    current_part_stock = deepcopy(process_state.part_stock)

    # for layout_state in completed_instructions.values():
    #     current_part_stock[0] -= len(layout_state.required_fit_positions)
    #     current_part_stock[layout_state.pipe_id] -= 1

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

    check_pos = list(detour_event)[0][-1]
    if check_pos in (start, goal):
        check_pos = list(detour_event)[0][0]

    for partial_solution in partial_solutions:
        for trail in partial_solution.ordered_trails:

            if check_pos in trail:
                solution = partial_solver.fuse_partial_solutions(partial_solutions, completed_instructions,
                                                                 initial_path_problem)
                break

    return solution


def make_registration_message(event_pos: Pos, event_code: int, removal: bool, pipe_id: int) -> str:
    """Returns a message as string confirming a placement or removal event.

    Args:
        event_pos (:obj:`~type_aliases.Pos`): See :paramref:`~ProcessPlanning.ProcessState.evaluate_placement.event_pos`
        event_code (:obj:`int`): See :paramref:`~ProcessState.ProcessState.evaluate_placement.event_code`

    Returns:
        :obj:`str`
        """

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


def make_special_message(note: str, event_pos: Pos) -> str:
    """Returns a special message as string, usually used in case of deviated placements."""
    message = str.format(f"Process Planner: Position {event_pos}: {note} ")
    return message


def make_error_message(event_pos: Pos, note: str) -> str:
    """Returns error messages as string containing the position where the error occurred as well as additional
    information regarding the reason for the error."""
    message = str.format(f"Process Planner: Process error at Position {event_pos}: {note}")
    return message


def get_next_recommended_action(process_state, building_instruction) -> Action:
    """Calculates the completion for the building instruction and determines the next recommended action.

    Args:
        building_instruction(:class:`BuildingInstruction`): Building instruction currently being followed.
        process_state(:class:`ProcessState`): The current process state.
    Returns:
        :obj:`~type_aliases.Action` containing the position, motion event code and part id for the next recommended action.
    """

    completed = process_state.completed_instruction(building_instruction)
    rec_pos = None
    rec_event = None
    rec_part_id = None

    if completed[1] == 3:
        rec_pos = building_instruction.recommended_attachment_pos
        rec_event = 3
        rec_part_id = -1

    if completed[1] == 2:
        # pipe next
        rec_event = 2
        rec_part_id = building_instruction.pipe_id
        # get attachment that is not deviated
        for pos in building_instruction.possible_att_pipe_positions:
            pos, construction_state = process_state.get_construction_state(pos, [3])
            if construction_state:
                if not construction_state.deviated:
                    rec_pos = pos
                    break
    elif completed[1] == 1:
        # fittings next
        rec_event = 1
        rec_part_id = constants.fitting_id

        for pos in building_instruction.required_fit_positions:
            # get required fit pos
            if None in process_state.get_construction_state(pos, [1]):
                rec_pos = pos
                break

    elif completed[0]:
        # layout complete, get next incomplete layout

        for layout in process_state.aimed_solution.ordered_trails:
            building_instruction = process_state.building_instructions.get(layout)
            if not building_instruction.layout_completed:
                return get_next_recommended_action(process_state, building_instruction)

    return rec_pos, rec_event, rec_part_id


def make_placement_messages(event_info: PlacementEventInfo) -> (str, str):
    """Produces a message according to the given event info.

    Args:
        event_info(:class:`PlacementEventInfo`): See :class:`PlacementEventInfo`

    Returns:
        :obj:`tuple` containing a message.
    """

    note = None

    if event_info.obstructed_obstacle:
        note = str.format(f"Obstructed obstacle while placing {message_dict[event_info.event_code]}")

    if event_info.obstructed_part:
        note = str.format(
            f"Obstructed {message_dict[event_info.obstructed_part]} while placing {message_dict[event_info.event_code]}")

    if event_info.removal:
        if event_info.unnecessary:
            note = str.format(f"Removed unnecessary {message_dict[event_info.event_code]}")
        elif event_info.misplaced:
            note = str.format(f"Removed misplaced {message_dict[event_info.event_code]}")
        elif event_info.deviated:
            note = str.format(f"Removed deviating {message_dict[event_info.event_code]}")
    else:
        if event_info.unnecessary:
            note = str.format(f"Unnecessary {message_dict[event_info.event_code]} detected!")
        elif event_info.deviated:
            note = str.format(f"Deviating {message_dict[event_info.event_code]} detected!")
        elif event_info.misplaced:
            note = str.format(f"Misplaced {message_dict[event_info.event_code]} detected!")

    if event_info.part_not_picked:
        if event_info.part_id == -99:
            note = str.format(f"Placed {message_dict[event_info.event_code]}"
                              f", but part was not picked!")
        else:
            note = str.format(f"Placed id {event_info.part_id} "
                              f", but not picked!")

    # make messages
    if event_info.error:
        message = make_error_message(event_pos=event_info.event_pos,
                                     note=note)
    elif event_info.deviated:
        message = make_registration_message(event_pos=event_info.event_pos, event_code=event_info.event_code,
                                            removal=event_info.removal, pipe_id=event_info.part_id)

    else:
        message = make_registration_message(event_pos=event_info.event_pos, event_code=event_info.event_code,
                                            removal=event_info.removal, pipe_id=event_info.part_id)

    return message, note


def make_pick_messages(event_info: PickEventInfo) -> (str, str):
    """Produces a message according to the given event info.

    Args:
        event_info(:class:`PickEventInfo`): See :class:`PickEventInfo`

    Returns:
        :obj:`tuple` containing a message.
    """
    message = None
    note = None
    if event_info.error:
        message = str.format(f"Error while picking part with ID {event_info.part_id}!")
        if event_info.part_not_available:
            note = str.format(f"Stock for part with ID {event_info.part_id} empty!")
    else:
        message = str.format(f"Process Planner: Picked part with ID {event_info.part_id}")

    return message, note


def get_valid_placement_positions(process_state: ProcessState, part_id: int) -> set[Union[Pos, Trail]]:
    """Returns all valid placement positions for the part that was picked.

    Args:
        process_state(:class:`ProcessState`): The process state after the motion event has been evaluated
        part_id(:obj:`int`): Part ID
    Returns:
        :obj:`set` [:obj:`Union` [:obj:`~type_aliases.Pos`,:obj:`~type_aliases.Trail`]]
    """
    building_instruction: BuildingInstruction
    valid_pos = set()
    if part_id:  # straight pipe
        for building_instruction in process_state.building_instructions.values():
            state = process_state.completed_instruction(building_instruction)
            if state[1] == constants.pipe_event_code and building_instruction.pipe_id == part_id:
                valid_pos.add(building_instruction.possible_att_pipe_positions)


    elif part_id == constants.fitting_id:
        placed_fit_pos_set = process_state.get_positions_from_motion_dict(constants.fit_event_code)
        for building_instruction in process_state.building_instructions.values():

            state = process_state.completed_instruction(building_instruction)
            if state[1] == constants.fit_event_code:
                req_fit_set = set(building_instruction.required_fit_positions)
                unplaced_fittings = req_fit_set.difference(placed_fit_pos_set)
                valid_pos.update(unplaced_fittings)

    return valid_pos
