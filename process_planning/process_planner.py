from __future__ import annotations

from copy import deepcopy
from typing import Optional

from path_finding.path_finding_util.path_math import get_direction, diff_pos
from path_finding.pf_data_class.path_problem import PathProblem
from path_finding.search_algorithm import find_path
from process_planning.pp_data_class.pick_event_result import PickEventResult
from process_planning.pp_data_class.assembly_event_result import AssemblyEventResult
from process_planning.pp_data_class.process_output import ProcessOutput
from process_planning.process_state import ProcessState
from process_planning.process_util import pp_util
from type_dictionary import constants
from type_dictionary.class_types import BuildingInstructions
from type_dictionary.type_aliases import *

picking_robot_command_message_dict = {
    -2: "STOP",
    -1: "Cancel all picking tasks",
    0: "Go to neutral state",
    1: "Move to pick-up point and pick part id: ",
    2: "Move to return position and return of part id: ",  # unused
    3: "Move to offering position",
    4: "Wait for worker to accept part",
}

fastening_robot_command_message_dict = {
    -2: "STOP",
    -1: "Remove fastening of positions x from command pipeline (if still in pipeline)",
    0: "Go to neutral state",
    1: "Move to a fastening position and fasten attachment at Position: ",
    2: "Move to a fastening position and fasten pipe at Position: "
}


# Todo:
#   Known Issues:
#   - Incorrect calculation of detour event in transition nodes (need to account for transition nodes here)


# noinspection PyUnboundLocalVariable
class ProcessPlanner:
    """Acts as an interface for handling events. Keeps track of the building process with the :class:`ProcessState<process_state>`
    class and provides robot commands. Handles calculation of new solutions in case of a detour event.
    """

    def __init__(self, initial_path_problem: PathProblem, initial_process_state: Optional[ProcessState] = None):
        """
        Args:
            initial_path_problem(:class:`PathProblem<path_problem>`): See :class:`PathProblem<path_problem>`.
            initial_process_state(:obj:`Optional` [:class:`PathProblem<path_problem>`]): Optional parameter if an initial process state exists.

        :ivar _initial_path_problem: A copy of the original path problem
        :ivar optimal_solution: A solution to the initial path problem.
        :ivar next_recommended_action: The next recommended worker action according to the process planner"""

        self._initial_path_problem = initial_path_problem
        self.optimal_solution = find_path(self._initial_path_problem)

        self.initial_process_state = initial_process_state

        self.next_recommended_action = None

        if self.optimal_solution is None:
            print("Process Planner Error: No optimal solution found!")
            if initial_process_state:
                if initial_process_state.aimed_solution:
                    self.optimal_solution = initial_process_state.aimed_solution
                    print("Process Planner: Assuming aimed solution from initial state as optimal solution.")
            else:
                raise Exception("No optimal solution found! Provide either a solvable path problem or an initial "
                                "process state!")
        else:
            if not initial_process_state:
                self.initial_process_state = ProcessState(self.optimal_solution)
                self.initial_process_state.aimed_solution = self.optimal_solution
                # set initial layout at start
                self.initial_process_state.last_event_trail = self.initial_process_state.aimed_solution.ordered_trails[
                    0]
                # get first recommended action
                first_building_instruction = self.initial_process_state.building_instructions.get(
                    self.initial_process_state.last_event_trail)
                self.next_recommended_action = (first_building_instruction.recommended_attachment_pos, 3, -1)

        self.previous_states = []  # contains all previous valid states

        self.last_process_state = self.initial_process_state  # last valid state
        self.last_output = None

    def handle_motion_event(self, motion_event: MotionEvent, handle_detour_events: bool = True,
                            ignore_part_check: bool = False, ignore_empty_stock: bool = False,
                            ignore_obstructions: bool = False) -> ProcessOutput:
        """Main method of ProcessPlanner. Takes a :obj:`~type_aliases.MotionEvent` as input and sends information about
        the event to a :class:`ProcessState  <process_state>` instance. Handles detour events and produces robot commands.

        Args:
            motion_event(:obj:`~type_aliases.MotionEvent`): A tuple containing event position and event code. Contains a
             part ID instead of a position in case of a pick event.
            handle_detour_events(:obj:`bool`): If set to true, detour events will result in the process planner looking
             for a new solution.
            ignore_part_check(:obj:`bool`): If set to true, part restrictions will be ignored. Could lead to unexpected
             behaviour.
            ignore_obstructions(:obj:`bool`): If set to true, obstructions will be ignored. Could lead to unexpected
             behaviour.
            ignore_empty_stock(:obj:`bool`): If set to true, parts with empty stock can be picked without error. Could
             lead to unexpected behaviour.

        Returns: :class:`ProcessOutput<process_output>` containing processed information regarding the event and
        current process state. """

        event_pos = motion_event[0]
        event_code = motion_event[1]

        tentative_process_state = deepcopy(self.last_process_state)
        picking_robot_commands = []
        fastening_robot_commands = []
        valid_assembly_positions = set()
        messages = ()

        self.previous_states.insert(0, deepcopy(self.last_process_state))

        if event_code in (constants.pick_manual_event_code, constants.pick_robot_event_code):
            # motion event was pick event
            picking_robot_commands = self.determine_picking_robot_commands(event_code=event_code,
                                                                      process_state=tentative_process_state)
            part_id = event_pos
            event_result: PickEventResult = tentative_process_state.pick_part(event_code, part_id, ignore_empty_stock)
            tentative_process_state.last_pick_event_result = event_result

            messages = pp_util.make_pick_messages(event_result)

            valid_assembly_positions = pp_util.get_valid_assembly_positions(process_state=tentative_process_state,
                                                                             part_id=part_id)

        elif event_code in {constants.fit_event_code, constants.pipe_event_code, constants.att_event_code}:
            # motion event was assembly event

            # check if worker event occurred on transition point, correct if necessary
            start_pos = self._initial_path_problem.start_pos
            for transition_point in self._initial_path_problem.transition_points:

                check_pos = None
                if event_pos[0] == transition_point[0]:
                    check_pos = (transition_point[0], start_pos[1])
                elif event_pos[1] == transition_point[1]:
                    check_pos = (start_pos[0], transition_point[1])

                if check_pos:
                    direction = get_direction(
                        diff_pos(self._initial_path_problem.start_pos,
                                 check_pos))  # get direction relative to start pos
                    # correct worker event pos
                    event_pos = (event_pos[0] - direction[0], event_pos[1] - direction[1])

            event_result: AssemblyEventResult = tentative_process_state.evaluate_assembly(event_pos=event_pos,
                                                                                          event_code=event_code,
                                                                                          ignore_part_check=ignore_part_check,
                                                                                          ignore_obstructions=ignore_obstructions)

            tentative_process_state.last_assembly_event_result = event_result

            messages = pp_util.make_assembly_messages(event_result=event_result)

            # extract messages
            message = messages[0]
            special_message = messages[1]

            detour_event: dict = tentative_process_state.last_assembly_event_result.detour_event

            # handle all detour events
            if detour_event and handle_detour_events:
                # new detour event
                detour_message, tentative_process_state = self.handle_detour_event(detour_event,
                                                                                   tentative_process_state)
            else:
                # handle existing detour events
                detour_message = self.handle_detour_trails(process_state=tentative_process_state)

            # get next recommended action
            building_instruction = tentative_process_state.building_instructions.get(
                tentative_process_state.last_event_trail)
            if building_instruction:
                self.next_recommended_action = pp_util.get_next_recommended_action(tentative_process_state,
                                                                                   building_instruction)

            # get fastening commands
            fastening_robot_commands = self.determine_fastening_robot_commands(event_pos=event_pos,
                                                                               event_code=event_code,
                                                                               event_result=tentative_process_state.last_assembly_event_result)
            # get picking robot commands
            picking_robot_commands = self.determine_picking_robot_commands(event_code=event_code,
                                                                      layout=tentative_process_state.last_assembly_event_result.layout,
                                                                      process_state=tentative_process_state)

            # get valid assembly positions
            if tentative_process_state.picked_parts:
                valid_assembly_positions = pp_util.get_valid_assembly_positions(tentative_process_state,
                                                                                 tentative_process_state.picked_parts[
                                                                                      0])

            messages = (message, special_message, detour_message)

        self.last_process_state = tentative_process_state

        process_output = ProcessOutput(process_state=tentative_process_state,
                                       current_event_result=event_result,
                                       messages=messages, next_recommended_action=self.next_recommended_action,
                                       valid_assembly_positions=valid_assembly_positions,
                                       picking_robot_commands=picking_robot_commands,
                                       fastening_robot_commands=fastening_robot_commands)

        self.last_output = process_output

        return process_output

    def handle_detour_trails(self, process_state: ProcessState) -> Optional[str]:
        """Handles current detour trails and decides if the currently aimed solution should return to a previous iteration.

        Args:
            process_state (:class:`ProcessState<process_state>`): The current process state.

        Returns:
            An optional :obj:`str` message if the currently aimed solution was changed to a previous one.
        """

        #todo: change condition for removing detour trail: remove both fittings and pipe

        detour_message = None
        if process_state.detour_trails:

            # check if previous layouts that caused detour are not completed anymore
            previous_detour_trails = deepcopy(process_state.detour_trails)
            last_detour_trail = previous_detour_trails.pop(-1)
            for detour_trail in previous_detour_trails:
                building_instruction = process_state.building_instructions[detour_trail]
                if process_state.completed_instruction(building_instruction)[1] >= constants.pipe_event_code:
                    process_state.detour_trails.remove(detour_trail)

            # check if layout that caused last detour is not completed anymore
            building_instruction = process_state.building_instructions[last_detour_trail]
            completed_instructions = pp_util.get_completed_instructions(process_state.building_instructions)

            if process_state.completed_instruction(building_instruction)[1] >= constants.pipe_event_code:
                process_state.detour_trails.pop(-1)
                if process_state.detour_trails and completed_instructions:
                    # there are still complete detour layouts
                    last_detour_trail = process_state.detour_trails[-1]

                    last_detour_instruction = process_state.building_instructions[last_detour_trail]
                    detour_event = {last_detour_trail: last_detour_instruction}
                    solution = pp_util.get_solution_on_detour_event(initial_path_problem=self._initial_path_problem,
                                                                    process_state=process_state,
                                                                    detour_event=detour_event)
                    process_state.reevaluate_motion_dict_from_solution(solution, detour_event)
                    detour_message = str.format(
                        f"Last deviated layout was removed. Returned to previous solution")
                else:
                    # return to optimal solution
                    detour_event = {None: "Returned to optimal solution"}
                    process_state.reevaluate_motion_dict_from_solution(self.optimal_solution)
                    process_state.detour_trails.clear()
                    detour_message = str.format(
                        f"No deviating layouts left. Returned to optimal solution.")

                process_state.last_assembly_event_result.detour_event = detour_event
        return detour_message

    def handle_detour_event(self, detour_event: BuildingInstructions, process_state: ProcessState) -> tuple[
        str, ProcessState]:
        """Handles the detour event, applies new solution if found.

        Args:
            process_state(:class:`ProcessState<process_state>`): The current process state.
            detour_event(:obj:`~class_types.BuildingInstructions`): Dictionary containing a trail and building instruction of the deviated layout.

        Returns:
            A :obj:`tuple` containing :obj:`str` messages and a modified state with the new solution applied (:obj:`tuple` [:obj:`str`, :class:`ProcessState<process_state>`]).

        """
        detour_message = str.format(f"Detour event confirmed, but no alternative solution found!")
        detour_process_state = deepcopy(process_state)
        detour_process_state.building_instructions.update(detour_event)

        # update state grid
        for pos in list(detour_event.keys())[0]:
            detour_process_state.state_grid[pos] = 2
        solution = pp_util.get_solution_on_detour_event(initial_path_problem=self._initial_path_problem,
                                                        process_state=detour_process_state,
                                                        detour_event=detour_event)
        if solution:
            detour_message = str.format(f"Detour event confirmed. Alternative solution was applied!")
            detour_trail = detour_process_state.reevaluate_motion_dict_from_solution(solution, detour_event)
            detour_process_state.detour_trails.append(detour_trail)
            # clean up remaining detour trails
            for detour_trail in detour_process_state.detour_trails:
                if not detour_trail in detour_process_state.building_instructions.keys():
                    # detour trail is not in current solution anymore
                    detour_process_state.detour_trails.remove(detour_trail)
        return detour_message, detour_process_state

    @staticmethod
    def determine_fastening_robot_commands(event_pos: Pos, event_code: int, event_result) -> tuple:
        """Reads the current process state and determines command codes for the fastening robot.

        Args:
            event_pos (:obj:`~type_aliases.Pos`): See parameter :paramref:`~process_state.ProcessState.evaluate_assembly.event_pos`
            event_code (:obj:`int`): See parameter :paramref:`~process_state.ProcessState.evaluate_assembly.event_code`
            event_result(:class:`AssemblyEventResult <assembly_event_result>`): See :class:`AssemblyEventResult <assembly_event_result>`
        Returns:
            :obj:`tuple` containing robot command codes for the fastening robot (See :ref:`Fastening Robot Command Codes`).

        """

        fastening_robot_commands = []
        # make picking robot commands

        # make fastening robot commands
        if not event_result.removal:
            if event_code == 3:
                fastening_robot_commands.append((1, event_pos))

            if event_code == 2:
                fastening_robot_commands.append((2, event_pos))

            # if fastening_robot_commands:
            #     print("Next fastening robot commands: ")
            #     for item in fastening_robot_commands:
            #         print(fastening_robot_command_message_dict[item[0]] + str(item[1]))
            #     print("\n")

        return tuple(fastening_robot_commands)

    @staticmethod
    def determine_picking_robot_commands(event_code: int, process_state: ProcessState,
                                         layout: Trail = None) -> tuple:
        """Reads the current process state and determines command codes for the picking robot.

        Args:
            process_state(:class:`ProcessState<process_state>`): The current process state.
            event_code (:obj:`int`): See parameter :paramref:`~process_state.ProcessState.evaluate_assembly.event_code`
            layout(:obj:`~type_aliases.Trail`): The current layout.
        Returns:
            :obj:`tuple` containing robot command codes for the picking robot (See :ref:`Picking Robot Command Codes`).
        """

        picking_robot_commands = []

        if layout:

            # make picking robot commands

            # if event_info.layout_changed:  # last check is redundant
            #     picking_robot_commands.append(-1)

            if event_code == 4:
                picking_robot_commands.append(0)

            next_part_id = pp_util.determine_next_part(process_state=process_state, layout=layout)

            if event_code == 3:
                if next_part_id:
                    picking_robot_commands.extend([(1, next_part_id), 3, 4])

            # print commands
            # if picking_robot_commands:
            #     print("Next picking robot commands: ")
            #     for item in picking_robot_commands:
            #         if isinstance(item, tuple):
            #             print(picking_robot_command_message_dict[item[0]] + str(item[1]))
            #         else:
            #             print(picking_robot_command_message_dict[item])
            #     print("\n")

        return tuple(picking_robot_commands)

    def return_to_previous_state(self):
        """Restores the previous state as the current state.

        Returns:
            :obj:`bool` if restoring the previous state was successful.
        """
        if self.previous_states:
            self.last_process_state = self.previous_states.pop(0)
            print("Returned to previous state!")
            return True
        else:
            print("There is no last state to return to!")
            return False
