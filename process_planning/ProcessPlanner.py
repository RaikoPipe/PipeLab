from process_planning.ProcessState import ProcessState
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from data_class.EventInfo import EventInfo
from data_class.Solution import Solution
from copy import deepcopy
from path_finding.search_algorithm import find_path
from typing import Optional
from path_finding import path_math, partial_solver
import numpy as np

from path_finding.common_types import *
from process_planning.pp_util import get_completed_layouts, get_outgoing_connections, get_outgoing_directions, \
    determine_next_part

standard_weights = Weights(1, 1, 1)
standard_algorithm = "mcsa*"

message_dict = {1: "fitting", 2: "pipe", 3: "attachment"}

example_motion_dict = {1: (1, 1)}  # considering motion capture speed, will probably never be bigger than 1

def get_neighboring_layouts(current_layout: Trail, layouts: Layouts) -> list[Trail]:
    neighboring_layouts = []
    idx = layouts.index(current_layout)

    if idx + 1 < len(layouts):
        neighboring_layouts.append(layouts[idx + 1])
    if idx - 1 >= 0:
        neighboring_layouts.append(layouts[idx - 1])

    return neighboring_layouts


# todo: multiple walls: one state --> need to reintroduce features to path finding
def adjust_pos_in_connections_set(connections_set, pos):
    """remove pos from first connection that contains it. If not found, then add as a connection containing only pos."""
    for connection in connections_set:
        if pos in connection:
            set_connection = set(connection)
            set_connection.discard(pos)
            connections_set.discard(connection)
            connections_set.add((set_connection.pop(), ()))
            break
    else:
        connections_set.add((pos, ()))


def get_solution_on_rerouting_event(initial_path_problem, process_state, rerouting_event) -> Optional[Solution]:
    completed_instructions = get_completed_layouts(process_state.building_instructions)

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

    layout_outgoing_connections_set = get_outgoing_connections(completed_instructions)
    layout_outgoing_directions_dict = get_outgoing_directions(completed_instructions)

    start = path_problem.start_pos
    goal = path_problem.goal_pos

    # Add start/goal to the connections set. If already in a connection, discard.
    adjust_pos_in_connections_set(layout_outgoing_connections_set, start)
    adjust_pos_in_connections_set(layout_outgoing_connections_set, goal)

    layout_outgoing_directions_dict[
        initial_path_problem.start_pos] = initial_path_problem.start_directions
    layout_outgoing_directions_dict[
        initial_path_problem.goal_pos] = initial_path_problem.goal_directions

    exclusion_list = [{start, goal}]  # contains connections that are excluded from partial solving

    # get new solution
    partial_solutions = partial_solver.get_partial_solutions(
        outgoing_connections_set=layout_outgoing_connections_set,
        outgoing_directions_dict=layout_outgoing_directions_dict,
        exclusion_list=exclusion_list,
        part_stock=current_part_stock,
        state_grid=current_state_grid,
        path_problem=path_problem)
    solution = None

    reroute_trail = list(rerouting_event)[0][0]

    for partial_solution in partial_solutions:
        if reroute_trail in partial_solution.layouts:
            solution = partial_solver.fuse_partial_solutions(partial_solutions, completed_instructions, initial_path_problem)
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
            f"process_planning: Registered {motion_type} for object {object_name} (ID {pipe_id}) at Position {event_pos}")
    else:
        message = str.format(
            f"process_planning: Registered {motion_type} for object {object_name} at Position {event_pos}")

    return message


def make_special_message(message: str, event_pos: Pos):
    """Returns a special message as string, usually used in case of deviated placements/removals."""
    message = str.format(f"process_planning: Position {event_pos}: {message} ")
    return message


def make_error_message(event_pos: Pos, additional_message: str):#
    """Returns error messages as string containing the position where the error occurred as well as additional
    information regarding the reason for the error."""
    message = str.format(f"process_planning: Process error at Position {event_pos}: {additional_message}")
    return message


class ProcessPlanner:
    """Acts as an interface for handling events. Keeps track of the building process with ProcessState and provides robot
     commands. Handles calculation of a new solution in case of a rerouting event."""

    def __init__(self, initial_path_problem: PathProblem, initial_process_state: Optional[ProcessState],
                 optimization_weights: Weights = standard_weights,
                 algorithm: str = standard_algorithm):

        self._initial_path_problem = initial_path_problem  # original path problem
        self.optimal_solution = find_path(self._initial_path_problem)  # optimal solution for the initial path problem


        self.initial_process_state = initial_process_state

        if self.optimal_solution is None:
            print("process_planning Error: No optimal solution found!")
            if initial_process_state:
                if initial_process_state.aimed_solution:
                    self.optimal_solution = initial_process_state.aimed_solution
                    print("process_planning: Assuming aimed solution from initial state as optimal solution.")
        else:
            if not initial_process_state:
                self.initial_process_state = ProcessState(self.optimal_solution)
                self.initial_process_state.aimed_solution = self.optimal_solution
                # set initial layout at start
                self.initial_process_state.last_event_trail = self.initial_process_state.aimed_solution.layouts[0]



        self.latest_deviation_solution = None

        self.previous_states = []  # contains all previous valid states

        self.latest_assembly_state = self.initial_process_state  # latest valid state

        self.tentative_process_state = deepcopy(self.initial_process_state)
        self.is_optimal = True  # if path of current state is on optimal solution

        self.weights = optimization_weights
        self.algorithm = algorithm

    def send_new_pick_event(self, part_id: int) -> str:
        """Sends a new pick event to be evaluated and registered in current ProcessState."""
        message = self.tentative_process_state.pick_part(part_id)
        print(message)
        self.previous_states.insert(0, deepcopy(self.tentative_process_state))
        return message

    def main(self, worker_event: tuple[Union[Pos,int], int], check_for_deviations: bool = True, ignore_errors: bool = True):
        """Main method of ProcessPlanner. Takes worker_event as input and sends information about the event to
        ProcessState. Prints message output of ProcessState and handles possible rerouting events. Returns current
         ProcessState, messages and robot commands to the picking robot and the fastening robot.
        Args:
            worker_event: A tuple containing event position and event code. Contains part ID instead of position in case
            of a pick event.
            """
        messages = self.send_placement_event(worker_event=worker_event,
                                             check_for_deviation_events=check_for_deviations,
                                             ignore_errors=ignore_errors)

        # extract messages
        message = messages[0]
        special_message = messages[1]
        error_message = messages[2]

        # print messages
        if message:
            print(message)
        if special_message:
            print(special_message)
        if error_message:
            print(error_message)

        rerouting_event = self.tentative_process_state.last_event_info.rerouting_event

        if rerouting_event:

            error_message = str.format(f"Rerouting event confirmed, but no alternative solution found!")

            solution = get_solution_on_rerouting_event(initial_path_problem=self._initial_path_problem,
                                                       process_state=self.tentative_process_state,
                                                       rerouting_event=rerouting_event)


            if solution:
                error_message = str.format(f"Rerouting event confirmed, applying alternative solution!")
                self.tentative_process_state.handle_rerouting_event(rerouting_event, solution)

        return self.tentative_process_state, (message,special_message,error_message)



    def send_placement_event(self, worker_event: tuple[Pos, int], check_for_deviation_events: bool = True,
                             ignore_errors: bool = False, allow_stacking: bool = False):
        """Sends a new placement/removal event to be evaluated and registered in current ProcessState."""

        self.previous_states.insert(0, deepcopy(self.tentative_process_state))

        event_info : EventInfo = self.tentative_process_state.evaluate_placement(worker_event=worker_event, check_for_deviation_events=check_for_deviation_events,
                                                        ignore_errors=ignore_errors, allow_stacking=allow_stacking)

        special_note = None
        error_note = None

        if event_info.obstructed_obstacle:
            error_note = str.format(f"Obstructed obstacle while placing {message_dict[event_info.event_code]}")
            error_message = make_error_message(event_pos=event_info.event_pos,
                                               additional_message=error_note)

        if event_info.obstructed_part:
            error_note = str.format(
                f"Obstructed {message_dict[event_info.event_code]} while placing {message_dict[event_info.obstructed_part]}")
            error_message = make_error_message(event_pos=event_info.event_pos,
                                               additional_message=error_note)
        if event_info.part_not_picked:
            if event_info.part_id == -99:
                error_note = str.format(f"Placed {message_dict[event_info.event_code]}"
                                        f", but part was not picked!")
            else:
                error_note = str.format(f"Placed id {event_info.part_id} "
                                        f", but not picked!")

        if event_info.removal:
            if event_info.unnecessary:
                special_note = str.format(f"Removed unnecessary {message_dict[event_info.event_code]}")

            elif event_info.misplaced:
                special_note = str.format(f"Removed misplaced {message_dict[event_info.event_code]}")
        else:
            if event_info.unnecessary:
                special_note = str.format(f"Unnecessary {message_dict[event_info.event_code]} detected!")
            if event_info.misplaced:
                special_note = str.format(f"Misplaced {message_dict[event_info.event_code]} detected!")


        message = None
        special_message = None
        error_message = None

        # make messages
        if not event_info.error:
            message = make_registration_message(event_pos=event_info.event_pos, event_code=event_info.event_code,
                                                removal=event_info.removal, pipe_id=event_info.part_id)

        if special_note:
            special_message = make_special_message(message=special_note, event_pos=event_info.event_pos)

        if error_note:
            error_message = make_error_message(event_pos=event_info.event_pos,
                                               additional_message=error_note)

        self.tentative_process_state.last_event_info = event_info

        #todo: remove debug
        print(self.tentative_process_state.motion_dict)

        return message, special_message, error_message

    def determine_robot_commands(self, worker_event: tuple[Pos, int], info_dict: dict) -> tuple[list, list]:
        """Evaluates the current process state and issues robot commands."""

        deviation_code = info_dict["deviation_code"]
        current_layout = info_dict["current_layout"]
        current_layout_state = info_dict["current_layout_state"]
        layout_changed = info_dict["layout_changed"]
        pipe_id = info_dict["pipe_id"]
        deviation = info_dict["deviation"]
        obsolete_parts = info_dict["obsolete_parts"]

        event_code = worker_event[0]
        event_pos = worker_event[1]

        retrieve_part_id = None

        fastening_robot_commands = []
        picking_robot_commands = []

        next_part_id = determine_next_part(layout_state=current_layout_state)

        # make picking robot commands

        if layout_changed and event_code != 4:  # last check is redundant
            if self.picking_robot_carries_part_id:
                picking_robot_commands.extend([-1, 3, 4])
            else:
                # robot doesn't carry part
                picking_robot_commands.extend([(1, next_part_id), 3, 4])

        if event_code == 4:
            if next_part_id:
                picking_robot_commands.extend([(1, next_part_id), 3, 4])

        # make fastening robot commands
        if event_code == 3:
            fastening_robot_commands.append((1, event_pos))

        if obsolete_parts["attachments"]:
            fastening_robot_commands.append((-1, obsolete_parts["attachments"]))

        return fastening_robot_commands, picking_robot_commands

    def return_to_previous_state(self):
        """Returns to previous state."""
        if self.previous_states:
            print("Latest action was undone!")
            self.tentative_process_state = self.latest_assembly_state = self.previous_states.pop(0)
        else:
            print("There is no last state to return to!")
