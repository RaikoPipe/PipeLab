from __future__ import annotations

from copy import deepcopy
from typing import Optional

from path_finding import partial_solver
from path_finding.path_finding_util.path_math import get_direction, diff_pos
from path_finding.pf_data_class.path_problem import PathProblem
from path_finding.pf_data_class.solution import Solution
from process_planning.pp_data_class.pick_event_result import PickEventResult
from process_planning.pp_data_class.assembly_event_result import AssemblyEventResult
from process_planning.process_state import ProcessState
from type_dictionary import constants
from type_dictionary.class_types import *
from type_dictionary.constants import horizontal_directions, vertical_directions

message_dict = {1: "fitting", 2: "pipe", 3: "attachment"}


def determine_next_part(process_state: ProcessState, layout: Trail) -> Optional[int]:
    """Determines which part should be picked next according to the current build progress.

    Args:
        process_state(:class:`ProcessState<process_state>`): The current process state.
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


def get_outgoing_node_pairs(building_instructions: BuildingInstructions) -> NodePairSet:
    """Returns all outgoing points in building_instructions as a connection. Interpolates them if they are
    connected.

    Args:
        building_instructions(:obj:`~class_types.BuildingInstructions`): See :paramref:`~process_state.ProcessState.building_instructions`.

    Returns:
        :obj:`~type_aliases.NodePairSet`

    """

    outgoing_node_pairs_set = set()
    # for layout_state in building_instructions.values():
    #     outgoing_node_pairs_set.add(layout_state.required_fit_positions)

    # layout_state = layout_state_list.pop(0)
    #
    # while layout_state_list:
    #     for other_layout_state in layout_state_list:
    #         fit_positions = layout_state.required_fit_positions
    #         other_fit_positions = other_layout_state.required_fit_positions
    #         fit_positions_set = set(fit_positions)
    #         other_fit_positions_set = set(other_fit_positions)
    #         intersection = fit_positions_set.intersection(other_fit_positions_set)
    #         if intersection:
    #             outgoing_node_pairs_set.discard(fit_positions)
    #             outgoing_node_pairs_set.discard(other_fit_positions)
    #
    #             intersected_pos = intersection.pop()
    #             fit_positions_set.discard(intersected_pos)
    #             other_fit_positions_set.discard(intersected_pos)
    #
    #             fits_left = (fit_positions_set.pop(), other_fit_positions_set.pop())
    #             new_end_points = (fits_left[0], fits_left[1])
    #
    #             outgoing_node_pairs_set.add(new_end_points)
    #     layout_state = layout_state_list.pop()

    instructions = [i for i in building_instructions.values()]

    connections = []
    for instruction in instructions:
        connections.append(set(instruction.required_fit_positions))

    connection_list = []
    while connections:
        c = [connections.pop()]
        recursion_search(c, connections)
        connection_list.append(c)

    final_list = []
    for c in connection_list:
        new_c = []
        for connection in c:
            counter = 0
            intersection_list = []
            for other_connection in c:
                intersect = connection.intersection(other_connection)

                if intersect:
                    if connection == other_connection:
                        continue
                    intersection_list.append(intersect)
                    counter += 1

            if counter <= 1:
                if intersection_list:
                    intersect = intersection_list[0]
                    cop = deepcopy(connection)
                    cop.remove(intersect.pop())
                    new_c.append(cop.pop())
                else:
                    cop = deepcopy(connection)
                    new_c.append(cop.pop())
                    new_c.append(cop.pop())

        final_list.append(new_c)
    outgoing_node_pairs_set = set()
    for l in final_list:
        outgoing_node_pairs_set.add(tuple(l))

    return outgoing_node_pairs_set


def recursion_search(c, connections):
    for connection in connections:
        appended = False
        for con in c:
            if con.intersection(connection):
                c.append(connection)
                appended = True
                break
        if appended:
            connections.remove(connection)
            recursion_search(c, connections)
            continue



def get_outgoing_node_directions(building_instructions: BuildingInstructions) -> FittingDirections:
    """Returns the connecting directions the fittings of each building instruction requires.

    Args:
        building_instructions(:obj:`~class_types.BuildingInstructions`): See :paramref:`~process_state.ProcessState.building_instructions`.

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


def adjust_pos_in_node_pairs_set(node_pair_set: NodePairSet, pos: Pos):
    """Adjusts the first tuple in "node_pair_set" containing the node position "pos" by removing it. If not found,
    it will add a tuple containing only "pos" and an empty tuple to "node_pair_set".
    This is to prevent the partial solver algorithm from connecting the node position of start or goal to other
    nodes later if they are already in a layout.


    Args:
        pos(:obj:`~type_aliases.Pos`): Pos to be evaluated.
        node_pair_set(:obj:`~type_aliases.NodePairSet`): Set with all outgoing node pairs.

    Returns:
        :obj:`~type_aliases.Pos` of the node the partial solver will use as the first node to start the search.
        """
    for connection in node_pair_set:
        if pos in connection:
            set_connection = set(connection)
            set_connection.discard(pos)
            node_pair_set.discard(connection)
            new_pair = (set_connection.pop(), ())
            node_pair_set.add(new_pair)
            return new_pair[0]
    else:
        pair = (pos, ())
        node_pair_set.add(pair)
        return pos


def get_solution_on_detour_event(initial_path_problem: PathProblem, process_state: ProcessState, detour_event) -> \
        Optional[Solution]:
    """Tries to get a new solution on a detour event.
    Args:

    initial_path_problem (:class:`PathProblem<path_problem>`): The original path problem.
    process_state (:class:`ProcessState<process_state>`): The current process state.

    Returns:
        :class:`Solution<solution>` if one was found, else None.
    """
    completed_instructions = process_state.get_completed_instructions()

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
    search_start_pos = adjust_pos_in_node_pairs_set(layout_outgoing_node_pairs_set, start)
    search_goal_pos = adjust_pos_in_node_pairs_set(layout_outgoing_node_pairs_set, goal)

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
        path_problem=path_problem,
        search_start_pos=search_start_pos,
        search_goal_pos=search_goal_pos
    )

    # get a position of the detouring layout
    check_pos = list(detour_event)[0][-1]
    if check_pos in (start, goal):
        check_pos = list(detour_event)[0][0]

    detour_layout_in = False
    goal_in = False
    # check if position of the detouring layout is in the partial solution
    for partial_solution in partial_solutions:
        for trail in partial_solution.ordered_trails:
            if check_pos in trail:
                detour_layout_in = True
            if goal in trail:
                goal_in = True
                break

    if detour_layout_in and goal_in:
        solution = partial_solver.fuse_partial_solutions(partial_solutions, completed_instructions,
                                                         initial_path_problem)
    else:
        solution = None

    return solution


def make_registration_message(event_pos: Pos, event_code: int, removal: bool, pipe_id: int) -> str:
    """Returns a message as string confirming a assembly or removal event.

    Args:
        event_pos (:obj:`~type_aliases.Pos`): See :paramref:`~process_planning.ProcessState.evaluate_assembly.event_pos`
        event_code (:obj:`int`): See :paramref:`~process_state.ProcessState.evaluate_assembly.event_code`

    Returns:
        :obj:`str`
        """

    object_name = message_dict[event_code]
    motion_type = "assembly"
    if removal:
        motion_type = "removal"

    if pipe_id not in (0, -1, None):
        message = str.format(
            f"{event_pos} Registered {motion_type} for object {object_name} (ID {pipe_id})")
    elif pipe_id == -2:
        message = str.format(
            f"{event_pos} Registered {motion_type} for object {object_name} (ID Unknown)")
    else:
        message = str.format(
            f"{event_pos} Registered {motion_type} for object {object_name}")

    return message


def make_special_message(note: str, event_pos: Pos) -> str:
    """Returns a special message as string, usually used in case of deviated assemblys."""
    message = str.format(f"{event_pos} Process deviation: {note} ")
    return message


def make_error_message(event_pos: Pos, note: str) -> str:
    """Returns error messages as string containing the position where the error occurred as well as additional
    information regarding the reason for the error."""
    message = str.format(f"{event_pos} Process error: {note}")
    return message


def get_next_recommended_action(process_state, building_instruction) -> Action:
    """Calculates the completion for the building instruction and determines the next recommended action.

    Args:
        building_instruction(:class:`BuildingInstruction<building_instruction>`): Building instruction currently being followed.
        process_state(:class:`ProcessState<process_state>`): The current process state.
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


def make_assembly_messages(event_result: AssemblyEventResult) -> (str, str):
    """Produces a message according to the given event result.

    Args:
        event_result(:class:`AssemblyEventResult <assembly_event_result>`): See :class:`AssemblyEventResult <assembly_event_result>`

    Returns:
        :obj:`tuple` containing a message.
    """

    note = None

    if event_result.obstructed_obstacle:
        note = str.format(f"Obstructed obstacle while placing {message_dict[event_result.event_code]}")

    if event_result.obstructed_part:
        note = str.format(
            f"Obstructed {message_dict[event_result.obstructed_part]} while placing {message_dict[event_result.event_code]}")

    if event_result.removal:
        if event_result.unnecessary:
            note = str.format(f"Removed unnecessary {message_dict[event_result.event_code]}")
        elif event_result.misplaced:
            note = str.format(f"Removed misplaced {message_dict[event_result.event_code]}")
        elif event_result.deviated:
            note = str.format(f"Removed deviating {message_dict[event_result.event_code]}")
    else:
        if event_result.unnecessary:
            note = str.format(f"Unnecessary {message_dict[event_result.event_code]} detected!")
        elif event_result.deviated:
            note = str.format(f"Deviating {message_dict[event_result.event_code]} detected!")
        elif event_result.misplaced:
            note = str.format(f"Misplaced {message_dict[event_result.event_code]} detected!")

    if event_result.part_not_picked:
        if event_result.part_id == -99:
            note = str.format(f"Placed {message_dict[event_result.event_code]}"
                              f", but part was not picked!")
        else:
            note = str.format(f"Placed id {event_result.part_id} "
                              f", but not picked!")

    # make messages
    if event_result.error:
        message = make_error_message(event_pos=event_result.event_pos,
                                     note=note)
    elif event_result.deviated:
        message = make_registration_message(event_pos=event_result.event_pos, event_code=event_result.event_code,
                                            removal=event_result.removal, pipe_id=event_result.part_id)

    else:
        message = make_registration_message(event_pos=event_result.event_pos, event_code=event_result.event_code,
                                            removal=event_result.removal, pipe_id=event_result.part_id)

    return message, note


def make_pick_messages(event_result: PickEventResult) -> (str, str):
    """Produces a message according to the given event result.

    Args:
        event_result(:class:`PickEventResult<pick_event_result>`): See :class:`PickEventResult<pick_event_result>`

    Returns:
        :obj:`tuple` containing a message.
    """
    message = None
    note = None
    if event_result.error:
        message = str.format(f"Error while picking part with ID {event_result.part_id}!")
        if event_result.part_not_available:
            note = str.format(f"Stock for part with ID {event_result.part_id} empty!")
    else:
        message = str.format(f"Process Planner: Picked part with ID {event_result.part_id}")

    return message, note


def get_valid_assembly_positions(process_state: ProcessState, part_id: int) -> set[Union[Pos, Trail]]:
    """Returns all valid assembly positions for the part that was picked.

    Args:
        process_state(:class:`ProcessState<process_state>`): The process state after the motion event has been evaluated
        part_id(:obj:`int`): Part ID
    Returns:
        :obj:`set` [:obj:`Union` [:obj:`~type_aliases.Pos`, :obj:`~type_aliases.Trail`]]
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
