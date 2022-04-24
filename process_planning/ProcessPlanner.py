from copy import deepcopy
from typing import Optional, Union

from data_class.EventInfo import EventInfo
from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
from data_class.Weights import Weights
from path_finding import partial_solver
from type_dictionary.common_types import *
from path_finding.search_algorithm import find_path
from process_planning.ProcessState import ProcessState
from process_planning.pp_util import get_completed_instructions, get_outgoing_node_pairs, get_outgoing_node_directions, \
    determine_next_part


standard_weights = Weights(1, 1, 1)
standard_algorithm = "mcsa*"

message_dict = {1: "fitting", 2: "pipe", 3: "attachment"}

example_motion_dict = {1: (1, 1)}  # considering motion capture speed, will probably never be bigger than 1


# todo: multiple walls: one state --> need to reintroduce features to path finding
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

# Todo:
#   Known Issues:
#   - Can't return to optimal solution (Fix: Track detour trail, if removed, go back to optimal solution)
#   - Pipes with deviating IDs can't be placed on solution nodes
#   - missing intelligence: if unknown pipe id in between two fittings (in a detour event), check if distance between two fittings
#       exists as a picked part, then assign and continue detour event

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

        self.latest_deviation_solution = None

        self.previous_states = []  # contains all previous valid states

        self.latest_assembly_state = self.initial_process_state  # latest valid state

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

    def main(self, worker_event: tuple[Union[Pos, int], int], check_for_deviations: bool = True,
             ignore_errors: bool = True):
        """Main method of ProcessPlanner. Takes worker_event as input and sends information about the event to
        ProcessState. Prints message output of ProcessState and handles possible detour events. Returns current
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

        # print messages
        if message:
            print(message)
        if special_message:
            print(special_message)

        detour_event = self.tentative_process_state.last_event_info.detour_event
        detour_message = None
        if detour_event:

            detour_message = str.format(f"detour event confirmed, but no alternative solution found!")

            solution = get_solution_on_detour_event(initial_path_problem=self._initial_path_problem,
                                                    process_state=self.tentative_process_state,
                                                    detour_event=detour_event)

            if solution:
                detour_message = str.format(f"detour event confirmed, applying alternative solution!")
                self.tentative_process_state.handle_detour_event(detour_event, solution)

        return self.tentative_process_state, (message, special_message, detour_message)

    def send_placement_event(self, worker_event: tuple[Pos, int], check_for_deviation_events: bool = True,
                             ignore_errors: bool = False, allow_stacking: bool = False):
        """Sends a new placement/removal event to be evaluated and registered in current ProcessState."""

        self.previous_states.insert(0, deepcopy(self.tentative_process_state))

        event_info: EventInfo = self.tentative_process_state.evaluate_placement(worker_event=worker_event,
                                                                                check_for_deviation_events=check_for_deviation_events,
                                                                                ignore_errors=ignore_errors,
                                                                                allow_stacking=allow_stacking)
        message = None
        special_message = None
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
            self.tentative_process_state = self.latest_assembly_state = self.previous_states.pop(0)
            print("Latest action was undone!")
        else:
            print("There is no last state to return to!")
