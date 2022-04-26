from copy import deepcopy
from typing import Union

from data_class.EventInfo import EventInfo
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from path_finding.path_math import get_direction, diff_pos
from type_dictionary.common_types import *
from path_finding.search_algorithm import find_path
from process_planning.ProcessState import ProcessState
from process_planning.pp_util import determine_next_part, get_solution_on_detour_event, make_registration_message, \
    make_error_message, message_dict

standard_weights = Weights(1, 1, 1)
standard_algorithm = "mcsa*"

picking_robot_command_message_dict = {-2: "STOP",
                                      -1: "Cancel all picking tasks",
                                      0: "Go to neutral state",
                                      1: "Move to pick-up point and pick part id: ",
                                      2: "Move to return position and return of part id: ", # unused
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
#   - missing intelligence: if unknown pipe id in between two fittings (in a detour event), check if distance between two fittings
#       exists as a picked part, then assign and continue detour event (add picked_parts_stamp to construction state:contains all parts picked before the event, then get difference between current parts and the stamp)
#   - add check: pipe placement only allowed if event on existing attachment, else error
class ProcessPlanner:
    """Acts as an interface for handling events. Keeps track of the building process with ProcessState and provides robot
     commands. Handles calculation of a new solution in case of a detour event."""

    def __init__(self, initial_path_problem: PathProblem, initial_process_state: Optional[ProcessState],
                 optimization_weights: Weights = standard_weights,
                 algorithm: str = standard_algorithm):

        self._initial_path_problem = initial_path_problem  # original path problem
        self.optimal_solution = find_path(self._initial_path_problem)  # optimal solution for the initial path problem

        self.initial_process_state = initial_process_state

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

        self.previous_states = []  # contains all previous valid states

        self.last_process_state = self.initial_process_state  # last valid state

        self.tentative_process_state = deepcopy(self.initial_process_state)
        self.is_optimal = True  # if path of current state is on optimal solution

        self.weights = optimization_weights
        self.algorithm = algorithm

    def send_new_pick_event(self, part_id: int) -> str:
        """Sends a new pick event to be evaluated and registered in current ProcessState."""
        self.previous_states.insert(0, deepcopy(self.tentative_process_state))
        message = self.tentative_process_state.pick_part(part_id)
        print(message)

        return message

    def main(self, worker_event: tuple[Union[Pos, int], int], handle_detour_events: bool = True,
             ignore_part_check: bool = True):
        """Main method of ProcessPlanner. Takes worker_event as input and sends information about the event to
        ProcessState. Prints message output of ProcessState and handles possible detour events. Returns current
         ProcessState, messages and robot commands.
         
        :param handle_detour_events: 
        :param worker_event: A tuple containing event position and event code. Contains part ID instead of position in case
        of a pick event.
        :param ignore_part_check: 
            """
        # check if worker event is pick event
        if worker_event[1] == 4:
            self.determine_picking_robot_commands(worker_event=worker_event)
            return self.tentative_process_state, (self.send_new_pick_event(worker_event[0]))


        # check if worker event occurred on transition point, correct if necessary
        start_pos = self._initial_path_problem.start_pos
        for transition_point in self._initial_path_problem.transition_points:

            check_pos = None
            if worker_event[0][0] == transition_point[0]:
                check_pos = (transition_point[0], start_pos[1])
            elif worker_event[0][1] == transition_point[1]:
                check_pos = (start_pos[0], transition_point[1])

            if check_pos:
                direction = get_direction(diff_pos(self._initial_path_problem.start_pos,check_pos)) # get direction relative to start pos
                # correct worker event pos
                worker_event = ((worker_event[0][0] - direction[0],worker_event[0][1] - direction[1]), worker_event[1])
        messages = self.send_placement_event(worker_event=worker_event,
                                             ignore_part_check=ignore_part_check)

        # extract messages
        message = messages[0]
        special_message = messages[1]

        # print messages
        if message:
            print(message)
        if special_message:
            print(special_message)

        detour_event : dict = self.tentative_process_state.last_event_info.detour_event
        detour_message = None

        if detour_event and handle_detour_events:
            self.tentative_process_state.detour_trails.append(list(detour_event)[0])
            detour_message = str.format(f"detour event confirmed, but no alternative solution found!")

            solution = get_solution_on_detour_event(initial_path_problem=self._initial_path_problem,
                                                    process_state=self.tentative_process_state,
                                                    detour_event=detour_event)

            if solution:
                detour_message = str.format(f"detour event confirmed, applying alternative solution!")
                self.tentative_process_state.handle_detour_event(solution, detour_event)

                # clean up remaining detour trails
                for detour_trail in self.tentative_process_state.detour_trails:
                    if not detour_trail in self.tentative_process_state.building_instructions.keys():
                        # detour trail is not in current solution anymore
                        self.tentative_process_state.detour_trails.remove(detour_trail)
        else:
            if self.tentative_process_state.detour_trails:
                # check if previous layouts that caused detour are not completed anymore
                previous_detour_trails = deepcopy(self.tentative_process_state.detour_trails)
                last_detour_trail = previous_detour_trails.pop(-1)
                for detour_trail in previous_detour_trails:
                    if not self.tentative_process_state.building_instructions[last_detour_trail].layout_completed:
                        self.tentative_process_state.detour_trails.remove(detour_trail)
                # check if layout that caused last detour is not completed anymore
                if not self.tentative_process_state.building_instructions[last_detour_trail].layout_completed:
                    self.tentative_process_state.detour_trails.pop(-1)
                    if self.tentative_process_state.detour_trails:
                        # there are still complete detour layouts
                        last_detour_trail = self.tentative_process_state.detour_trails[-1]

                        last_detour_instruction = self.tentative_process_state.building_instructions[last_detour_trail]
                        detour_event = {last_detour_trail:last_detour_instruction}
                        solution = get_solution_on_detour_event(initial_path_problem=self._initial_path_problem,
                                                                process_state=self.tentative_process_state,
                                                                detour_event=detour_event)
                        self.tentative_process_state.handle_detour_event(solution, detour_event)
                        detour_message = str.format(
                            f"Deviating layout incomplete, returning solution for last deviating layout")
                    else:
                        # return to optimal solution
                        detour_event = {None: "Returned to optimal solution"}
                        self.tentative_process_state.handle_detour_event(self.optimal_solution)


                        detour_message = str.format(f"No complete deviating layouts left, returning to optimal solution")

                    self.tentative_process_state.last_event_info.detour_event = detour_event

        # get fastening commands
        self.determine_fastening_robot_commands(worker_event=worker_event, event_info=self.tentative_process_state.last_event_info)


        self.determine_picking_robot_commands(worker_event=worker_event,
                                              layout=self.tentative_process_state.last_event_info.layout)


        return self.tentative_process_state, (message, special_message, detour_message)

    def send_placement_event(self, worker_event: tuple[Pos, int],
                             ignore_part_check: bool = False, ignore_obstructions: bool = False):
        """Sends a new placement/removal event to be evaluated and registered in current ProcessState."""

        self.previous_states.insert(0, deepcopy(self.tentative_process_state))

        event_info: EventInfo = self.tentative_process_state.evaluate_placement(worker_event=worker_event,
                                                                                ignore_part_check=ignore_part_check,
                                                                                ignore_obstructions=ignore_obstructions)

        note = None


        if event_info.obstructed_obstacle:
            note = str.format(f"Obstructed obstacle while placing {message_dict[event_info.event_code]}")

        if event_info.obstructed_part:
            note = str.format(
                f"Obstructed {message_dict[event_info.event_code]} while placing {message_dict[event_info.obstructed_part]}")



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

        self.tentative_process_state.last_event_info = event_info

        return message, note

    @staticmethod
    def determine_fastening_robot_commands(worker_event: tuple[Pos, int], event_info):

        event_code = worker_event[1]
        event_pos = worker_event[0]

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

        return fastening_robot_commands


    def determine_picking_robot_commands(self, worker_event: tuple[Pos, int], layout: Trail = None):
        """Evaluates the current process state and issues robot commands."""

        picking_robot_commands = []

        if layout:

            event_code = worker_event[1]
            # make picking robot commands

            # if event_info.layout_changed:  # last check is redundant
            #     picking_robot_commands.append(-1)

            if event_code == 4:
                picking_robot_commands.append(0)

            next_part_id = determine_next_part(process_state=self.tentative_process_state, layout=layout)

            if event_code == 3:
                if next_part_id:
                    picking_robot_commands.extend([(1, next_part_id), 3, 4])

            # print commands
            if picking_robot_commands:
                print("Next picking robot commands: ")
                for item in picking_robot_commands:
                    if isinstance(item, tuple):
                        print(picking_robot_command_message_dict[item[0]] + str(item[1]))
                    else:
                        print(picking_robot_command_message_dict[item])
                print("\n")


        return picking_robot_commands


    def return_to_previous_state(self):
        """Returns to previous state."""
        if self.previous_states:
            self.tentative_process_state = self.last_process_state = self.previous_states.pop(0)
            print("Latest action was undone!")
        else:
            print("There is no last state to return to!")
