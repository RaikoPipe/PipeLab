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
from process_planning.pp_utilities import get_completed_layouts, get_outgoing_connections, get_outgoing_directions, \
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
    message = str.format(f"process_planning: Position {event_pos}: {message} ")
    return message


def make_error_message(event_pos: Pos, additional_message: str):
    message = str.format(f"process_planning: Process error at Position {event_pos}: {additional_message}")
    return message


class ProcessPlanner:
    """Acts as an interface for handling events. Keeps track of the building process and provides new solutions on events. Returns instructions."""

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
                self.initial_process_state.last_event_layout = self.initial_process_state.aimed_solution.layouts[0]



        self.latest_deviation_solution = None

        self.previous_states = []  # contains all previous valid states

        self.latest_assembly_state = self.initial_process_state  # latest valid state

        self.tentative_process_state = deepcopy(self.initial_process_state)
        self.is_optimal = True  # if path of current state is on optimal solution

        self.weights = optimization_weights
        self.algorithm = algorithm

    def new_pick_event(self, part_id: int) -> str:
        message = self.tentative_process_state.pick_part(part_id)
        print(message)
        self.previous_states.insert(0, deepcopy(self.tentative_process_state))
        return message

    def main_func(self, worker_event: tuple[Pos, int], check_for_deviations: bool = True, ignore_errors: bool = True):
        messages = self.new_placement_event(worker_event=worker_event,
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



    def new_placement_event(self, worker_event: tuple[Pos, int], check_for_deviation_events: bool = True,
                              ignore_errors: bool = False, allow_stacking: bool = False):

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
        """Evaluates the current process state and issues robot commands"""

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

    # def deviation_event(self, worker_event, tentative_state: ProcessState, debug_grid: Optional[np.ndarray]) -> dict:
    #
    #     """Checks for deviating layouts on captured motion events and updates tentative state."""
    #
    #     event_code = worker_event[1]
    #     event_pos = worker_event[0]
    #
    #     check_order = [self.tentative_process_state.deviated_motion_set_fitting, self.tentative_process_state.motion_set_fitting]
    #
    #     if event_code == 1:
    #         for check_fit_set in check_order:
    #             for pos in check_fit_set:
    #
    #                 # check if conditions for a deviation event are met
    #
    #                 fit_diff = path_math.get_length_same_axis(pos,
    #                                                           event_pos)  # distance between fittings
    #                 pipe_id = fit_diff - 1  # length of needed part
    #
    #                 if pipe_id == 0:
    #                     # fittings are directly next to each other
    #                     break
    #
    #                 fittings_in_proximity = pipe_id in self.tentative_process_state.part_stock.keys()  # fittings are connectable by available parts
    #
    #                 if not fittings_in_proximity:
    #                     continue
    #
    #                 fit_dir = get_direction(diff_pos(pos, event_pos))
    #                 fit_tup = (pos, event_pos)
    #                 deviation_trail = get_deviation_trail(length=fit_diff, direction=fit_dir, fit_pos=fit_tup)
    #
    #                 attachment_is_between = False
    #
    #                 att_set = set()
    #                 pipe_set = set()
    #
    #                 pipe_trail = [i for i in deviation_trail if i not in fit_tup]
    #                 for att_pos in tentative_state.deviated_motion_set_attachment:
    #                     if att_pos in pipe_trail:
    #                         att_set.add(att_pos)
    #
    #                 if not att_set:
    #                     # no attachments in between
    #                     continue
    #
    #                 for pipe_pos in pipe_trail:
    #                     if tentative_state.deviated_motion_dict_pipe.get(pipe_pos) == pipe_id:
    #                         pipe_set.add(pipe_pos)
    #                     elif tentative_state.deviated_motion_dict_pipe.get(pipe_pos) == -1:
    #                         pipe_set.add(pipe_pos)
    #
    #                 if not pipe_set:
    #                     # no pipes in between
    #                     continue
    #
    #                 print("Deviation confirmed")
    #
    #                 first_pipe_pos = ()
    #                 # get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, event_pos, pos)
    #
    #                 deviation_state = get_deviation_state(length=fit_diff, att_set=att_set, pipe_set=pipe_set,
    #                                                       fit_tup=fit_tup,
    #                                                       state_grid=self._initial_path_problem.state_grid)
    #
    #                 tentative_state.fc_set.add(fit_tup)
    #
    #                 return {deviation_trail: deviation_state}

    def update_latest_state(self, old_state, new_state):
        self.previous_states.insert(0, old_state)

        self.latest_assembly_state = new_state

    def return_to_previous_state(self):
        """Returns to the last valid state"""
        if self.previous_states:
            print("Latest action was undone!")
            self.tentative_process_state = self.latest_assembly_state = self.previous_states.pop(0)
        else:
            print("There is no last state to return to!")
