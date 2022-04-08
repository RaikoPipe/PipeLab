import numpy as np
from data_class.AssemblyState import State
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from copy import deepcopy
from path_finding.search_algorithm import find_path
from path_finding.path_math import diff_pos, get_direction
from typing import Optional
from path_finding import path_math, partial_solver
from pp_utilities import *
from path_finding.common_types import *
from constants import valid_directions

from pp_utilities import get_deviation_trail, get_deviation_state

standard_weights = Weights(1, 1, 1)
standard_algorithm = "mcsa*"

message_dict = {1: "fitting", 2: "pipe", 3: "attachment"}


# create new data type list[tuple] with tuple = (pos, part_id)

def get_current_part_stock(part_stock: dict, parts_used: list) -> dict:
    """Returns the part stock sans the parts already used."""
    for part_id in parts_used:
        part_stock[part_id] -= 1

    return part_stock


example_motion_dict = {1: (1, 1)}  # considering motion capture speed, will probably never be bigger than 1


def get_neighboring_layouts(current_layout: Trail, layouts: Layouts) -> list[Trail]:
    neighboring_layouts = []
    idx = layouts.index(current_layout)

    if idx + 1 < len(layouts):
        neighboring_layouts.append(layouts[idx + 1])
    if idx - 1 >= 0:
        neighboring_layouts.append(layouts[idx - 1])

    return neighboring_layouts


def deconstruction_event(motion_events: dict, tentative_state: State, debug_grid: Optional[np.ndarray]) -> bool:
    """Checks for deconstruction events on captured motion events and updates tentative state."""

    for event_pos, event in motion_events.items():

        if event == 0:
            if event_pos in tentative_state.deviated_motion_set_fitting:

                for fit_set in tentative_state.fc_set:
                    if event_pos in fit_set:
                        tentative_state.fc_set.remove(fit_set)
                        tentative_state.removed_fc_set.add(fit_set)
                        tentative_state.deviated_motion_set_fitting.remove(event_pos)
                        break

            else:
                for trail in tentative_state.construction_layouts.keys():
                    if event_pos in trail:
                        if tentative_state.construction_layouts[trail] > 0:
                            tentative_state.construction_layouts[trail] -= 1
                    if tentative_state.construction_layouts[trail] == 0:
                        # construction has been fully removed
                        tentative_state.removed_fc_set.remove({trail[0], trail[-1]})

            print("Deconstruction confirmed")

            # get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, new_pos, pos)

            return True


def update_state_grid(state_grid, layout, completed):
    if completed:
        for pos in layout:
            state_grid[pos] = 2
    elif not completed:
        for pos in layout:
            state_grid[pos] = 0


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


def get_solution_on_deviation(initial_path_problem, process_state) -> Optional[Solution]:
    completed_layouts = get_completed_layouts(process_state.construction_layouts)
    print(completed_layouts)

    path_problem = deepcopy(initial_path_problem)

    # get state grid of completed layouts
    current_state_grid = path_problem.state_grid

    for layout_state in completed_layouts.keys():
        for pos in layout_state:
            current_state_grid[pos] = 2

    # get part stock (assuming only completed layouts)
    current_part_stock = path_problem.part_stock

    for layout_state in completed_layouts.values():
        current_part_stock[0] -= len(layout_state.fit_set)
        current_part_stock[layout_state.pipe_id] -= len(layout_state.pipe_set)

    layout_outgoing_connections_set = get_outgoing_connections(completed_layouts)
    layout_outgoing_directions_dict = get_outgoing_directions(completed_layouts)

    start = path_problem.start_pos
    goal = path_problem.goal_pos

    # Add start/goal to the connections set. If already in a connection, discard.
    adjust_pos_in_connections_set(layout_outgoing_connections_set, start)
    adjust_pos_in_connections_set(layout_outgoing_connections_set, goal)

    layout_outgoing_directions_dict[
        initial_path_problem.start_pos] = initial_path_problem.start_directions
    layout_outgoing_directions_dict[
        initial_path_problem.goal_pos] = initial_path_problem.goal_directions

    exclusion_list = []  # contains connections that are excluded from partial solving

    # get new solution
    partial_solutions = partial_solver.get_partial_solutions(
        outgoing_connections_set=layout_outgoing_connections_set,
        outgoing_directions_dict=layout_outgoing_directions_dict,
        exclusion_list=exclusion_list,
        part_stock=current_part_stock,
        state_grid=current_state_grid,
        path_problem=path_problem)
    solution = None
    if partial_solutions:
        solution = partial_solver.fuse_partial_solutions(partial_solutions, completed_layouts, initial_path_problem)

    return solution


class ProcessPlanner:
    """Acts as an interface for handling events. Keeps track of the building process and provides new solutions on events. Returns instructions."""

    def __init__(self, initial_path_problem: PathProblem, initial_state: Optional[State],
                 optimization_weights: Weights = standard_weights,
                 algorithm: str = standard_algorithm):

        self._initial_path_problem = initial_path_problem  # original path problem
        self.optimal_solution = find_path(self._initial_path_problem)  # optimal solution for the initial path problem
        if self.optimal_solution is None:
            print("ProcessPlanner Error: No optimal solution found!")
            if initial_state.aimed_solution:
                self.optimal_solution = initial_state.aimed_solution
                print("ProcessPlanner: Assuming aimed solution from initial state as optimal solution.")
        self.latest_deviation_solution = None

        self.previous_states = []  # contains all previous valid states

        if initial_state is None or initial_state.aimed_solution is None:
            initial_state = prepare_initial_state(solution=self.optimal_solution)
            # todo: add as debug feature (infinite picked parts)
            # for part_id in initial_state.picked_parts.keys():
            #     initial_state.picked_parts[part_id] = 999

        self.latest_state = initial_state  # latest valid state

        self.tentative_state = deepcopy(initial_state)
        self.is_optimal = True  # if path of current state is on optimal solution
        self.tentative_state.latest_layout = self.latest_state.aimed_solution.layouts[
            0]  # layout that is currently being built. Initial layout is at start.

        self.weights = optimization_weights
        self.algorithm = algorithm

        self.motion_dict = {}

        self.debug_motion_grid = self._initial_path_problem.state_grid

        # picking robot state
        self.picking_robot_carries_part_id = None

    def make_registration_message(self, event_pos: Pos, event_code: int, removal: bool, pipe_id: int) -> str:
        object_name = message_dict[event_code]
        motion_type = "placement"
        if removal:
            motion_type = "removal"

        if pipe_id not in (0, -1, None):
            message = str.format(
                f"ProcessPlanner: Registered {motion_type} for object {object_name} (ID {pipe_id}) at Position {event_pos}")
        else:
            message = str.format(
                f"ProcessPlanner: Registered {motion_type} for object {object_name} at Position {event_pos}")

        return message

    def make_special_message(self, message: str, event_pos: Pos):
        message = str.format(f"ProcessPlanner: Position {event_pos}: {message} ")
        return message

    def make_error_message(self, event_pos: Pos, additional_message: str):
        message = str.format(f"ProcessPlanner: Possible detection error at Position {event_pos}: {additional_message}")
        return message

    def new_pick_event(self, part_id: int) -> str:
        self.tentative_state.picked_parts.append(part_id)
        self.tentative_state.part_stock[part_id] -= 1
        message = str.format(f"ProcessPlanner: Picked part with id {part_id}")
        print(message)
        self.previous_states.insert(0, deepcopy(self.tentative_state))
        return message

    def main_func(self, worker_event: tuple[Pos, int], check_for_deviations: bool = True, ignore_errors: bool = True):
        process_state = self.evaluate_worker_event(worker_event=worker_event,
                                                   check_for_deviation_events=check_for_deviations,
                                                   ignore_errors=ignore_errors)

        # todo: if a deviation event occurred, use new_construction_check to evaluate already built layouts
        #   --> scan through layouts/built parts and extract worker_event_pos and worker_event code from them
        #   --> put them in a list and iterate them through new_construction_check

        event_info = process_state.event_info

        # extract messages
        message = event_info.get("message")
        special_message = event_info.get("special_message")
        error_message = event_info.get("error_message")

        # print messages
        if message:
            print(message)
        if special_message:
            print(special_message)
        if error_message:
            print(error_message)

        if event_info.get("deviation"):
            # handle deviation event

            solution = get_solution_on_deviation(initial_path_problem=self._initial_path_problem,
                                                 process_state=self.tentative_state)

            if solution:
                # todo: reevaluate motions -> inside solution, outside solution
                pass
            else:
                # todo: return error message
                pass

            self.tentative_state.aimed_solution = solution

        return self.tentative_state

    def evaluate_worker_event(self, worker_event: tuple[Pos, int], check_for_deviation_events: bool = True,
                              ignore_errors: bool = False, allow_stacking: bool = False):

        # todo: refactor: deepcopy latest state before this func, return tentative state from this func

        self.previous_states.insert(0, deepcopy(self.tentative_state))

        worker_event_pos = worker_event[0]
        worker_event_code = worker_event[1]

        pipe_id = None
        current_layout = None
        current_layout_state = None
        layout_changed = None
        deviation_code = None
        obsolete_parts = {}
        removal = False
        deviation = None
        deviated_motion = False
        message = None
        special_message = None
        error_message = None

        special_note = None
        error_note = None

        completed = None

        """deviation code
        -1: detection error: part was placed on an occupied spot
        0: process deviation: unnecessary part inside solution
        1: process deviation: part was placed incorrectly inside solution
        2: process deviation: part was placed outside solution
        3: process deviation: part was placed, but never picked prior"""

        # event evaluation

        current_layout = self.tentative_state.latest_layout
        current_layout_state = self.tentative_state.construction_layouts[current_layout]

        motion_dict = {1: self.tentative_state.motion_set_fitting, 2: self.tentative_state.motion_set_pipe,
                       3: self.tentative_state.motion_set_attachment}

        if not allow_stacking:

            if self.tentative_state.state_grid[worker_event_pos] == 1:
                error_note = str.format(f"Obstructed obstacle while placing {message_dict[worker_event_code]}")
                error_message = self.make_error_message(event_pos=worker_event_pos,
                                                        additional_message=error_note)
                event_info = {"error_message": error_message}
                self.tentative_state.event_info = event_info
                return self.tentative_state

            for event_code in motion_dict.keys():
                if worker_event_code == event_code or (worker_event_code == 2 and event_code == 3):
                    continue
                elif worker_event_pos in motion_dict[event_code]:
                    error_note = str.format(
                        f"Obstructed {message_dict[event_code]} while placing {message_dict[worker_event_code]}")
                    error_message = self.make_error_message(event_pos=worker_event_pos,
                                                            additional_message=error_note)
                    event_info = {"error_message": error_message}
                    self.tentative_state.event_info = event_info
                    return self.tentative_state

        # check if worker event was to remove misplaced/unnecessary parts
        if self.tentative_state.unnecessary_parts.get(worker_event_pos) == worker_event_code:
            special_note = str.format(f"Removed unnecessary {message_dict[worker_event_code]}")
            removal = True
            self.tentative_state.unnecessary_parts.pop(worker_event_pos)

        elif self.tentative_state.misplaced_parts.get(worker_event_pos) == worker_event_code:
            special_note = str.format(f"Removed misplaced {message_dict[worker_event_code]}")
            removal = True
            self.tentative_state.misplaced_parts.pop(worker_event_pos)

        # check if worker event occurred inside optimal solution
        elif worker_event_pos in self.tentative_state.aimed_solution.total_definite_trail.keys():

            # get information about layout where event occurred
            if worker_event_pos in self.tentative_state.latest_layout:
                current_layout = self.tentative_state.latest_layout
            else:
                trail = None
                # find the current layout
                if worker_event_code == 1:
                    for trail in self.tentative_state.construction_layouts.keys():
                        # current layout can be ambiguous
                        if worker_event_pos in self.tentative_state.construction_layouts[trail].required_fit_positions:
                            break
                    else:
                        for trail in self.tentative_state.construction_layouts.keys():
                            if worker_event_pos in trail:
                                break
                else:
                    for trail in self.tentative_state.construction_layouts.keys():
                        if worker_event_pos in trail:
                            break
                current_layout = trail
                self.tentative_state.latest_layout = current_layout
            layout_changed = current_layout != self.tentative_state.latest_layout

            current_layout_state = self.tentative_state.construction_layouts[current_layout]

            # todo: check recommended att pos

            # worker event evaluation on optimal solution
            if worker_event_code == 3:
                if worker_event_pos not in current_layout_state.required_fit_positions:
                    if not current_layout_state.att_set:
                        # successful placement
                        current_layout_state.att_set.add(worker_event_pos)
                    else:
                        # Unneeded attachment detected!
                        if not worker_event_pos in current_layout_state.att_set:
                            # attachment is unnecessary
                            deviation_code = 2
                            special_note = str.format(f"Unnecessary {message_dict[worker_event_code]} detected!")
                            self.tentative_state.unnecessary_parts[worker_event_pos] = 3

                        else:
                            # removed attachment
                            removal = True
                            current_layout_state.att_set.remove(worker_event_pos)
                else:
                    deviation_code = 1
                    special_note = str.format(f"Misplaced {message_dict[worker_event_code]} detected!")
                    self.tentative_state.misplaced_parts[worker_event_pos] = worker_event_code

            elif worker_event_code == 2:
                pipe_id = current_layout_state.pipe_id
                if not current_layout_state.pipe_set:
                    # check if part was actually picked
                    if pipe_id in self.tentative_state.picked_parts:
                        if worker_event_pos in current_layout_state.required_fit_positions:
                            special_note = str.format(f"Misplaced {message_dict[worker_event_code]} detected!")
                            self.tentative_state.misplaced_parts[worker_event_pos] = worker_event_code
                        # successful placement
                        self.tentative_state.picked_parts.remove(pipe_id)
                        current_layout_state.pipe_set.add(worker_event_pos)
                    else:
                        # ERROR : part was not picked!
                        deviation_code = 3
                        error_note = str.format(f"Part with id {pipe_id} "
                                                f"was placed, but not picked!")
                        # self.tentative_state.error_dict[worker_event_pos] = current_layout_state.pipe_id
                else:
                    # removed pipe
                    removal = True
                    current_layout_state.pipe_set.remove(worker_event_pos)

            elif worker_event_code == 1:
                # fixme: add special conditions to start and goal (is it a transition point?)
                pipe_id = 0
                if len(
                        current_layout_state.fit_set) < 2 and worker_event_pos in current_layout_state.required_fit_positions and worker_event_pos not in current_layout_state.fit_set:
                    if 0 in self.tentative_state.picked_parts:
                        # successful placement on correct pos
                        self.tentative_state.picked_parts.remove(0)
                        current_layout_state.fit_set.add(worker_event_pos)
                        neighboring_layouts = get_neighboring_layouts(current_layout,
                                                                      self.tentative_state.aimed_solution.layouts)
                        for layout in neighboring_layouts:
                            if worker_event_pos in layout:
                                self.tentative_state.construction_layouts[layout].fit_set.add(worker_event_pos)

                    else:
                        # ERROR : part was not picked!
                        deviation_code = 3
                        error_note = str.format(f"Part with id {0} "
                                                f"was placed, but not picked!")
                else:

                    if not worker_event_pos in current_layout_state.fit_set:
                        # Misplaced fitting detected!
                        if 0 in self.tentative_state.picked_parts:
                            deviation_code = 1
                            special_note = str.format(f"Misplaced {message_dict[worker_event_code]} detected!")
                            self.tentative_state.picked_parts.remove(0)
                            self.tentative_state.misplaced_parts[worker_event_pos] = worker_event_code
                        else:
                            # ERROR : part was not picked!
                            deviation_code = 3
                            error_note = str.format(f"Part with id {0} "
                                                    f"was placed, but not picked!")
                    else:
                        # removed fitting
                        removal = True
                        self.tentative_state.picked_parts.append(0)
                        current_layout_state.fit_set.remove(worker_event_pos)
                        neighboring_layouts = get_neighboring_layouts(current_layout,
                                                                      self.tentative_state.aimed_solution.layouts)
                        for layout in neighboring_layouts:
                            if worker_event_pos in layout:
                                self.tentative_state.construction_layouts[layout].fit_set.discard(worker_event_pos)

            completed = self.set_completion_state(current_layout, self.tentative_state.construction_layouts, removal)

        # worker event evaluation outside optimal solution
        elif check_for_deviation_events:
            # motion event occurred outside optimal solution
            deviated_motion = True
            deviation_code = 2
            if worker_event_code == 3:
                if worker_event_pos not in self.tentative_state.deviated_motion_set_attachment:
                    self.tentative_state.deviated_motion_set_attachment.add(worker_event_pos)
                    special_note = str.format(f"Deviated {message_dict[worker_event_code]} detected!")
                else:
                    self.tentative_state.deviated_motion_set_attachment.remove(worker_event_pos)
                    special_note = str.format(f"Removed deviated {message_dict[worker_event_code]}")
                    removal = True
            elif worker_event_code == 2:
                if worker_event_pos not in self.tentative_state.deviated_motion_set_pipe.keys():
                    picked_parts = self.tentative_state.picked_parts
                    picked_pipes = [i for i in picked_parts if i != 0]
                    if len(set(picked_pipes)) == 1:
                        # if there is only one of a kind of pipe in picked_parts, then we know which id was placed
                        pipe_id = picked_pipes[0]
                        picked_parts.remove(pipe_id)
                        self.tentative_state.deviated_motion_set_pipe[worker_event_pos] = pipe_id
                        special_note = str.format(f"Deviated {message_dict[worker_event_code]} detected!")
                    elif len(set(picked_parts)) > 1:
                        # if there are multiple kinds of pipe picked
                        self.tentative_state.deviated_motion_set_pipe[worker_event_pos] = -1
                        special_note = str.format(f"Deviated {message_dict[worker_event_code]} detected!")
                    else:
                        # ERROR : part was not picked!
                        deviation_code = 3
                        # special_note = str.format(f"Deviated {message_dict[worker_event_code]} detected!")
                        error_note = str.format(f"Part with unknown ID "
                                                f"was placed, but not picked!")
                        # self.tentative_state.error_dict[worker_event_pos] = current_layout_state.pipe_id

                else:
                    # removal
                    pipe_id = self.tentative_state.deviated_motion_set_pipe.pop(worker_event_pos)
                    special_note = str.format(f"Removed deviated {message_dict[worker_event_code]}")
                    removal = True

            elif worker_event_code == 1:
                if worker_event_pos not in self.tentative_state.deviated_motion_set_fitting:
                    if 0 in self.tentative_state.picked_parts:
                        special_note = str.format(f"Deviated {message_dict[worker_event_code]} detected!")
                        # check for deviation events

                        deviation = self.deviation_event(worker_event, self.tentative_state, None)
                        self.tentative_state.deviated_motion_set_fitting.add(worker_event_pos)
                        if deviation:
                            self.tentative_state.construction_layouts.update(deviation)
                            # update state grid
                            for pos in list(deviation.keys())[0]:
                                self.tentative_state.state_grid[pos] = 2
                    else:
                        # ERROR : part was not picked!
                        deviation_code = 3
                        error_note = str.format(f"Part with id {0} "
                                                f"was placed, but not picked!")
                else:
                    self.tentative_state.deviated_motion_set_fitting.remove(worker_event_pos)
                    special_note = str.format(f"Removed deviated {message_dict[worker_event_code]}")
                    removal = True

                    # todo: for rendering: visualization code should keep a reference of rendered objects pointing to the pos

        # return removed parts, update motion sets
        if not error_note:
            if removal:
                motion_dict[worker_event_code].remove(worker_event_pos)
                if worker_event_code == 1:
                    self.tentative_state.picked_parts.append(0)
                    pipe_id = 0
                elif worker_event_code == 2:
                    if not deviated_motion:
                        self.tentative_state.picked_parts.append(pipe_id)
                    else:
                        if pipe_id != -1:
                            self.tentative_state.picked_parts.append(pipe_id)
            else:
                motion_dict[worker_event_code].add(worker_event_pos)

        # make messages
        message = self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code,
                                                 removal=removal, pipe_id=pipe_id)
        if special_note:
            special_message = self.make_special_message(message=special_note, event_pos=worker_event_pos)

        if error_note:
            error_message = self.make_error_message(event_pos=worker_event_pos,
                                                    additional_message=error_note)

        # todo: output tentative state together with message instead of event_info
        # generate output
        event_info = {"current_layout": current_layout, "layout_changed": layout_changed,
                      "deviation_code": deviation_code, "pipe_id": pipe_id, "layout_state": current_layout_state,
                      "deviation": deviation, "obsolete_parts": obsolete_parts, "removal": removal, "message": message,
                      "special_message": special_message,
                      "error_message": error_message}

        self.tentative_state.event_info = event_info

        return self.tentative_state

    def set_completion_state(self, current_layout, construction_layouts, removal):
        """sets completion state of current layout and neighboring layouts. Updates state grid."""
        neighboring_layouts = get_neighboring_layouts(current_layout, self.tentative_state.aimed_solution.layouts)
        neighboring_layouts.append(current_layout)

        for layout in neighboring_layouts:
            layout_state = construction_layouts.get(layout)
            if not layout_state.completed:
                if layout_state.fit_set == set(
                        layout_state.required_fit_positions) and layout_state.pipe_set and layout_state.att_set:
                    # layout was completed
                    layout_state.completed = True
                    #self.tentative_state.fc_set.add((current_layout[0], current_layout[-1]))
                    for pos in layout:
                        self.tentative_state.state_grid[pos] = 2

            else:
                if removal:
                    layout_state.completed = False
                    #self.tentative_state.fc_set.remove((current_layout[0], current_layout[-1]))
                    for pos in layout:
                        self.tentative_state.state_grid[pos] = 0

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

    def deviation_event(self, worker_event, tentative_state: State, debug_grid: Optional[np.ndarray]) -> dict:

        """Checks for deviating layouts on captured motion events and updates tentative state."""

        event_code = worker_event[1]
        event_pos = worker_event[0]

        check_order = [self.tentative_state.deviated_motion_set_fitting, self.tentative_state.motion_set_fitting]

        if event_code == 1:
            for check_fit_set in check_order:
                for pos in check_fit_set:

                    # check if conditions for a deviation event are met

                    fit_diff = path_math.get_length_same_axis(pos,
                                                              event_pos)  # distance between fittings
                    pipe_id = fit_diff - 1  # length of needed part

                    if pipe_id == 0:
                        # fittings are directly next to each other
                        break

                    fittings_in_proximity = pipe_id in self.tentative_state.part_stock.keys()  # fittings are connectable by available parts

                    if not fittings_in_proximity:
                        continue

                    fit_dir = get_direction(diff_pos(pos, event_pos))
                    fit_tup = (pos, event_pos)
                    deviation_trail = get_deviation_trail(length=fit_diff, direction=fit_dir, fit_pos=fit_tup)

                    attachment_is_between = False

                    att_set = set()
                    pipe_set = set()

                    pipe_trail = [i for i in deviation_trail if i not in fit_tup]
                    for att_pos in tentative_state.deviated_motion_set_attachment:
                        if att_pos in pipe_trail:
                            att_set.add(att_pos)

                    if not att_set:
                        # no attachments in between
                        continue

                    for pipe_pos in pipe_trail:
                        if tentative_state.deviated_motion_set_pipe.get(pipe_pos) == pipe_id:
                            pipe_set.add(pipe_pos)
                        elif tentative_state.deviated_motion_set_pipe.get(pipe_pos) == -1:
                            pipe_set.add(pipe_pos)

                    if not pipe_set:
                        # no pipes in between
                        continue

                    print("Deviation confirmed")

                    first_pipe_pos = ()
                    # get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, event_pos, pos)

                    deviation_state = get_deviation_state(length=fit_diff, att_set=att_set, pipe_set=pipe_set,
                                                          fit_tup=fit_tup,
                                                          state_grid=self._initial_path_problem.state_grid)

                    tentative_state.fc_set.add(fit_tup)

                    return {deviation_trail: deviation_state}

    def update_latest_state(self, old_state, new_state):
        self.previous_states.insert(0, old_state)

        self.latest_state = new_state

    def return_to_previous_state(self):
        """Returns to the last valid state"""
        if self.previous_states:
            print("Latest action was undone!")
            self.tentative_state = self.latest_state = self.previous_states.pop(0)
        else:
            print("There is no last state to return to!")
