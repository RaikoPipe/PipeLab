from __future__ import annotations

from copy import deepcopy
from typing import Optional

from PathFinding.path_finding_util.path_math import get_direction, diff_pos
from PathFinding.pf_data_class.PathProblem import PathProblem
from PathFinding.pf_data_class.Weights import Weights
from PathFinding.search_algorithm import find_path
from ProcessPlanning.ProcessState import ProcessState
from ProcessPlanning.pp_data_class.EventInfo import EventInfo
from ProcessPlanning.pp_data_class.ProcessOutput import ProcessOutput
from ProcessPlanning.process_planning_util.pp_util import determine_next_part, get_solution_on_detour_event, \
    make_registration_message, \
    make_error_message, message_dict, get_next_recommended_action
from type_dictionary.common_types import *
from type_dictionary.special_types import MotionEvent, BuildingInstructions

standard_weights = Weights(1, 1, 1)
standard_algorithm = "mcsa*"

picking_robot_command_message_dict = {-2: "STOP",
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

example_motion_dict = {1: (1, 1)}  # considering motion capture speed, will probably never be bigger than 1


# Todo:
#   Known Issues:
#   -
#   Planned Features:
#   - Highlighting correct build spots
#   - check score calculation of mcsa*
#   - Output next recommended action


class ProcessPlanner:
    """Acts as an interface for handling events. Keeps track of the building process with the:class:`~ProcessState`
    class and provides robot commands.
    Handles calculation of a new solution in case of a detour event.

    :param initial_path_problem: Todo: Documentation
    :param initial_process_state: Todo: Documentation
    :param optimization_weights: Weights used if search algorithm is a multi-criteria search algorithm (mca*, mcsa*)
    :param algorithm: The search algorithm to be used for calculating any solutions to a path problem.
    """

    def __init__(self, initial_path_problem: PathProblem, initial_process_state: ProcessState = None,
                 optimization_weights: Weights = standard_weights,
                 algorithm: str = standard_algorithm):

        self._initial_path_problem = initial_path_problem  #: original path problem
        self.optimal_solution = find_path(self._initial_path_problem)  #: optimal solution for the initial path problem

        self.initial_process_state = initial_process_state

        self.next_recommended_action = None

        if self.optimal_solution is None:
            print("Process Planner Error: No optimal solution found!")
            if initial_process_state:
                if initial_process_state.aimed_solution:
                    self.optimal_solution = initial_process_state.aimed_solution
                    print("Process Planner: Assuming aimed solution from initial state as optimal solution.")
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

        self.is_optimal = True  # if path of current state is on optimal solution

        self.weights = optimization_weights
        self.algorithm = algorithm

    def main(self, motion_event: MotionEvent, handle_detour_events: bool = True,
             ignore_part_check: bool = False, ignore_obstructions: bool = False) -> ProcessOutput:
        """Main method of ProcessPlanning. Takes worker_event as input and sends information about the event to
        ProcessState. Prints message output of ProcessState and handles detour events.

        :param motion_event: A tuple containing event position and event code. Contains a part ID instead of a position in case of a pick event.
        :param handle_detour_events: If set to true, detour events will result in the process planner looking for a new solution.
        :param ignore_part_check: If set to true, part restrictions will be ignored. Could lead to unexpected behaviour.
        :param ignore_obstructions: If set to true, obstructions will be ignored. Untested. Could lead to unexpected behaviour.
        :return: A dataclass containing processed information regarding the event and current process state (See :class:`ProcessOutput`)."""

        tentative_process_state = deepcopy(self.last_process_state)
        # check if worker event is pick event
        if motion_event[1] == 4:
            picking_robot_commands = self.determine_picking_robot_commands(worker_event=motion_event,
                                                                           process_state=tentative_process_state)
            messages = self.send_pick_event(motion_event[0], process_state=tentative_process_state)
            self.previous_states.insert(0, deepcopy(tentative_process_state))
            self.last_process_state = tentative_process_state

            process_output = ProcessOutput(process_state=tentative_process_state,
                                           event_info=tentative_process_state.last_event_info,
                                           messages=(messages,), next_recommended_action=self.next_recommended_action,
                                           highlight_positions=(), picking_robot_commands=picking_robot_commands,
                                           fastening_robot_commands=())

            return process_output

        # check if worker event occurred on transition point, correct if necessary
        start_pos = self._initial_path_problem.start_pos
        for transition_point in self._initial_path_problem.transition_points:

            check_pos = None
            if motion_event[0][0] == transition_point[0]:
                check_pos = (transition_point[0], start_pos[1])
            elif motion_event[0][1] == transition_point[1]:
                check_pos = (start_pos[0], transition_point[1])

            if check_pos:
                direction = get_direction(
                    diff_pos(self._initial_path_problem.start_pos, check_pos))  # get direction relative to start pos
                # correct worker event pos
                motion_event = ((motion_event[0][0] - direction[0], motion_event[0][1] - direction[1]), motion_event[1])

        messages = self.send_placement_event(event_pos=motion_event[0], event_code=motion_event[1],
                                             ignore_part_check=ignore_part_check, process_state=tentative_process_state,
                                             ignore_obstructions=ignore_obstructions)

        # extract messages
        message = messages[0]
        special_message = messages[1]

        detour_event: dict = tentative_process_state.last_event_info.detour_event

        # handle all detour events
        if detour_event and handle_detour_events:
            detour_message, tentative_process_state = self.handle_detour_event(detour_event, tentative_process_state)
        else:
            detour_message = self.handle_detour_trails(process_state=tentative_process_state)

        # get next recommended action
        building_instruction = tentative_process_state.building_instructions.get(
            tentative_process_state.last_event_trail)
        if building_instruction:
            self.next_recommended_action = get_next_recommended_action(tentative_process_state,
                                                                       building_instruction)

        # get fastening commands
        fastening_robot_commands = self.determine_fastening_robot_commands(event_pos=motion_event[0],
                                                                           event_code=motion_event[1],
                                                                           event_info=tentative_process_state.last_event_info)

        picking_robot_commands = self.determine_picking_robot_commands(worker_event=motion_event,
                                                                       layout=tentative_process_state.last_event_info.layout,
                                                                       process_state=tentative_process_state)

        messages = (message, special_message, detour_message)

        self.previous_states.insert(0, deepcopy(tentative_process_state))
        self.last_process_state = tentative_process_state

        process_output = ProcessOutput(process_state=tentative_process_state,
                                       event_info=tentative_process_state.last_event_info,
                                       messages=messages, next_recommended_action=self.next_recommended_action,
                                       highlight_positions=(),
                                       picking_robot_commands=picking_robot_commands,
                                       fastening_robot_commands=fastening_robot_commands)

        return process_output

    @staticmethod
    def send_placement_event(event_pos: Pos, event_code:int, process_state: ProcessState,
                             ignore_part_check: bool = False, ignore_obstructions: bool = False):
        """Sends a new placement/removal event to be evaluated and registered in current ProcessState."""

        event_info: EventInfo = process_state.evaluate_placement(event_pos=event_pos, event_code=event_code,
                                                                 ignore_part_check=ignore_part_check,
                                                                 ignore_obstructions=ignore_obstructions)

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
                                         additional_message=note)
        elif event_info.deviated:
            message = make_registration_message(event_pos=event_info.event_pos, event_code=event_info.event_code,
                                                removal=event_info.removal, pipe_id=event_info.part_id)

        else:
            message = make_registration_message(event_pos=event_info.event_pos, event_code=event_info.event_code,
                                                removal=event_info.removal, pipe_id=event_info.part_id)

        process_state.last_event_info = event_info

        return message, note

    @staticmethod
    def send_pick_event(part_id: int, process_state: ProcessState) -> str:
        """Sends a new pick event to be evaluated and registered in the current process state.

        :param part_id: Part ID that was picked.
        :param process_state: The current process state.
        :return: A string containing a success message."""
        process_state.pick_part(part_id)
        message = str.format(f"Process Planner: Picked part with id {part_id}")

        return message

    def handle_detour_trails(self, process_state: ProcessState) -> Optional[str]:
        """Handles current detour trails and decides if the currently aimed solution should return to a previous iteration.

        :param process_state: The current process state
        :return: An optional string message if the currently aimed solution was changed to a previous one.
        """

        detour_message = None
        if process_state.detour_trails:
            # check if previous layouts that caused detour are not completed anymore
            previous_detour_trails = deepcopy(process_state.detour_trails)
            last_detour_trail = previous_detour_trails.pop(-1)
            for detour_trail in previous_detour_trails:
                if not process_state.building_instructions[detour_trail].layout_completed:
                    process_state.detour_trails.remove(detour_trail)
            # check if layout that caused last detour is not completed anymore
            if not process_state.building_instructions[last_detour_trail].layout_completed:
                process_state.detour_trails.pop(-1)
                if process_state.detour_trails:
                    # there are still complete detour layouts
                    last_detour_trail = process_state.detour_trails[-1]

                    last_detour_instruction = process_state.building_instructions[last_detour_trail]
                    detour_event = {last_detour_trail: last_detour_instruction}
                    solution = get_solution_on_detour_event(initial_path_problem=self._initial_path_problem,
                                                            process_state=process_state,
                                                            detour_event=detour_event)
                    process_state.adjust_motion_dict_to_solution(solution, detour_event)
                    detour_message = str.format(
                        f"Deviating layout incomplete, returning to solution for last deviating layout.")
                else:
                    # return to optimal solution
                    detour_event = {None: "Returned to optimal solution"}
                    process_state.adjust_motion_dict_to_solution(self.optimal_solution)

                    detour_message = str.format(
                        f"No complete deviating layouts left, returning to optimal solution.")

                process_state.last_event_info.detour_event = detour_event
        return detour_message

    def handle_detour_event(self, detour_event: BuildingInstructions, process_state: ProcessState) -> tuple[
        str, ProcessState]:
        """Handles the detour event, applies new solution if found.

        :param process_state: The current process state.
        :param detour_event: Dictionary containing a trail and building instruction of the deviated layout.
        :return: A tuple containing a string message and a modified state with the new solution applied.
        """
        detour_message = str.format(f"detour event confirmed, but no alternative solution found!")
        detour_process_state = deepcopy(process_state)
        detour_process_state.building_instructions.update(detour_event)
        detour_process_state.detour_trails.append(list(detour_event)[0])
        # update state grid
        for pos in list(detour_event.keys())[0]:
            detour_process_state.state_grid[pos] = 2
        solution = get_solution_on_detour_event(initial_path_problem=self._initial_path_problem,
                                                process_state=detour_process_state,
                                                detour_event=detour_event)
        if solution:
            detour_message = str.format(f"detour event confirmed, applying alternative solution!")
            detour_process_state.adjust_motion_dict_to_solution(solution, detour_event)

            # clean up remaining detour trails
            for detour_trail in detour_process_state.detour_trails:
                if not detour_trail in detour_process_state.building_instructions.keys():
                    # detour trail is not in current solution anymore
                    detour_process_state.detour_trails.remove(detour_trail)
        return detour_message, detour_process_state

    @staticmethod
    def determine_fastening_robot_commands(event_pos: Pos, event_code: int, event_info):

        retrieve_part_id = None

        fastening_robot_commands = []
        # make picking robot commands

        # make fastening robot commands
        if not event_info.removal:
            if event_code == 3:
                fastening_robot_commands.append((1, event_pos))

            if event_code == 2:
                fastening_robot_commands.append((2, event_pos))

            if fastening_robot_commands:
                print("Next fastening robot commands: ")
                for item in fastening_robot_commands:
                    print(fastening_robot_command_message_dict[item[0]] + str(item[1]))
                print("\n")

        return tuple(fastening_robot_commands)

    @staticmethod
    def determine_picking_robot_commands(worker_event: tuple[Pos, int], process_state: ProcessState,
                                         layout: Trail = None, ) -> tuple:
        """Evaluates the current process state and makes robot commands.

        :param layout: Optional last event layout if worker event code is 3.
        :param process_state: The current process state.
        :param worker_event: See :obj:`ProcessPlanner.worker_event`.
        """

        picking_robot_commands = []

        if layout:

            event_code = worker_event[1]
            # make picking robot commands

            # if event_info.layout_changed:  # last check is redundant
            #     picking_robot_commands.append(-1)

            if event_code == 4:
                picking_robot_commands.append(0)

            next_part_id = determine_next_part(process_state=process_state, layout=layout)

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
        """Returns to previous state."""
        if self.previous_states:
            self.last_process_state = self.previous_states.pop(0)
            print("Returned to previous state!")
        else:
            print("There is no last state to return to!")
