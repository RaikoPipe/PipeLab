import numpy as np
from data_class.State import State
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from copy import deepcopy
from path_finding.search_algorithm import find_path
from path_finding.path_math import diff_pos, get_direction
from typing import Optional
from path_finding import path_math, partial_solutionizer
from pp_utilities import *
from path_finding.common_types import *

from pp_utilities import get_deviation_trail, get_deviation_state

standard_weights = Weights(1, 1, 1)
standard_algorithm = "mcsa*"

message_dict = {1:"fitting", 2:"pipe", 3:"attachment"}

# create new data type list[tuple] with tuple = (pos, part_id)

def get_current_part_stock(part_stock: dict, parts_used: list) -> dict:
    """Returns the part stock sans the parts already used."""
    for part_id in parts_used:
        part_stock[part_id] -= 1

    return part_stock

example_motion_dict = {1: (1, 1)}  # considering motion capture speed, will probably never be bigger than 1


def get_neighboring_layouts(current_layout: Trail, layouts: Layouts):

    idx = layouts.index(current_layout)

    return {layouts[idx + 1] if idx + 1 < len(
        layouts) else None,
            layouts[idx - 1] if idx - 1 < 0 else None}


def deconstruction_event(motion_events: dict, tentative_state: State, debug_grid: Optional[np.ndarray]) -> bool:

    """Checks for deconstruction events on captured motion events and updates tentative state."""

    for event_pos, event in motion_events.items():

        if event == 0:
            if event_pos in tentative_state.deviated_motion_pos_fitting:

                for fit_set in tentative_state.fc_set:
                    if event_pos in fit_set:
                        tentative_state.fc_set.remove(fit_set)
                        tentative_state.removed_fc_set.add(fit_set)
                        tentative_state.deviated_motion_pos_fitting.remove(event_pos)
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

            #get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, new_pos, pos)

            return True


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


        self.latest_state = initial_state  # latest valid state

        self.tentative_state = deepcopy(initial_state)
        self.is_optimal = True  # if path of current state is on optimal solution
        self.tentative_state.latest_layout = self.latest_state.aimed_solution.layouts[0] # layout that is currently being built. Initial layout is at start.


        self.weights = optimization_weights
        self.algorithm = algorithm

        self.motion_dict = {}

        self.debug_motion_grid = self._initial_path_problem.state_grid

        # picking robot state
        self.picking_robot_carries_part_id = None

    def make_registration_message(self, event_pos:Pos, event_code:int, removal: bool):
        object_name = message_dict[event_code]
        motion_type = "placement"
        if removal:
            motion_type = "removal"

        print(str.format(f"ProcessPlanner: Registered {motion_type} for object {object_name} at Position {event_pos}"))

    def make_special_message(self, message:str, event_pos:Pos):
        print(str.format(f"ProcessPlanner: Position {event_pos}: {message} "))

    def make_error_message(self, event_pos:Pos, additional_message:str):
        print(str.format(f"ProcessPlanner: Possible detection error at Position {event_pos}: {additional_message}"))

    def new_pick_event(self, part_id:int):
        self.tentative_state.picked_parts[part_id] += 1
        print(str.format(f"ProcessPlanner: Picked part with id {part_id}"))

    def new_construction_check(self, worker_event: tuple[Optional[int], Pos]):


        worker_event_pos = worker_event[1]
        worker_event_code = worker_event[0]

        pipe_id = None
        current_layout = None
        current_layout_state = None
        layout_changed = None
        deviation_code = None
        obsolete_parts = {}
        removal = False

        """deviation code
        -1: detection error: part was placed on an occupied spot
        0: process deviation: unnecessary part inside solution
        1: process deviation: part was placed incorrectly inside solution
        2: process deviation: part was placed outside solution
        3: process deviation: part was placed, but never picked prior"""



        if worker_event_code == 4:
            #todo: add rendering instruction: highlight placement positions
            pass


        # todo: don't forget to clean up all the other detected motions + finished layouts outside!
        if worker_event_pos in self.tentative_state.aimed_solution.total_definite_trail.keys():
            # get information about the current layout
            for trail in self.tentative_state.construction_layouts.keys():
                if worker_event_pos in trail:
                    current_layout = trail

            layout_changed = current_layout != self.tentative_state.latest_layout

            current_layout_state = self.tentative_state.construction_layouts[current_layout]


            #todo: check recommended att pos
            if worker_event_code == 3:
                self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code, removal=False)
                if not current_layout_state.att_set:
                    # successful placement
                    current_layout_state.att_set.add(worker_event_pos)
                else:
                    # Unneeded attachment detected!
                    if not worker_event_pos in current_layout_state.att_set:
                        # attachment is unnecessary
                        deviation_code = 2
                        self.make_special_message(message="Unnecessary attachment detected at: ", event_pos=worker_event_pos)
                        self.tentative_state.remove_parts[worker_event_pos] = -1

                    else:
                        # removed attachment
                        removal = True
                        self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code, removal=True)
                        current_layout_state.att_set.remove(worker_event_pos)

            elif worker_event_code == 2:
                self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code, removal=False)
                pipe_id = current_layout_state.pipe_id
                if not current_layout_state.pipe_set:
                    # check if part was actually picked
                    if self.tentative_state.picked_parts[pipe_id] > 0:
                        # successful placement
                        self.tentative_state.picked_parts[pipe_id] -= 1
                        current_layout_state.pipe_set.add(worker_event_pos)
                    else:
                        # part was not picked!
                        deviation_code = 3
                        self.make_error_message(event_pos=worker_event_pos,
                                                additional_message=
                                                str.format(f"Part with id {current_layout_state.pipe_set} "
                                                           f"was placed, but not picked!"))
                        self.tentative_state.error_dict[worker_event_pos] = current_layout_state.pipe_id
                else:
                    # removed pipe
                    removal = True
                    self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code,
                                                   removal=True)
                    current_layout_state.pipe_set.clear()

            elif worker_event_code == 1:
                pipe_id = 0
                self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code, removal=False)
                if current_layout_state.fit_set < 2 and worker_event_pos in current_layout_state.correct_fitting_pos:
                    if self.tentative_state.picked_parts[0] > 0:
                        # successful placement on correct pos
                        self.tentative_state.picked_parts[0] -= 1
                        current_layout_state.fit_set.add(worker_event_pos)
                        neighboring_layouts = get_neighboring_layouts(current_layout, self.tentative_state.aimed_solution.layouts)

                        for layout in neighboring_layouts:
                            if worker_event_pos in layout:
                                self.tentative_state.construction_layouts[layout].fitting_pos = worker_event_pos

                    else:
                        # part was not picked!
                        deviation_code = 3
                        self.make_error_message(event_pos=worker_event_pos,
                                                additional_message=
                                                str.format(f"Part with id {0} "
                                                           f"was placed, but not picked!"))



                else:
                    # Unneeded fitting detected!
                    if not worker_event_pos in current_layout_state.att_set:
                        # fitting is unnecessary
                        deviation_code = 1
                        self.make_special_message(message="Unnecessary fitting detected at: ", event_pos=worker_event_pos)
                        self.tentative_state.remove_parts[worker_event_pos] = 0
                    else:
                        # removed fitting
                        removal = True
                        self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code, removal=True)
                        current_layout_state.fit_set.remove(worker_event_pos)

            self.set_completion_state(current_layout, removal)

            #update layout states of construction layouts
            self.tentative_state.construction_layouts[current_layout] = current_layout_state

        else:
            # motion event occurred outside optimal solution
            deviation_code = 2
            if worker_event_code == 3:
                if worker_event_pos not in self.tentative_state.deviated_motion_pos_attachment:
                    self.tentative_state.deviated_motion_pos_attachment.add(worker_event_pos)
                else: self.tentative_state.deviated_motion_pos_attachment.remove(worker_event_pos)
            elif worker_event_code == 2:
                if worker_event_pos not in self.tentative_state.deviated_motion_pos_pipe:
                    self.tentative_state.deviated_motion_pos_pipe.add(worker_event_pos)
                else:
                    self.tentative_state.deviated_motion_pos_pipe.remove(worker_event_pos)
            elif worker_event_code == 1:
                # check for deviation events
                deviation = self.deviation_event(worker_event, self.tentative_state, None)
                if deviation:
                    self.tentative_state.construction_layouts.update(deviation)
                    obsolete_fittings = set()
                    obsolete_attachments = set()
                    obsolete_pipes = set()
                    current_state_grid = self.tentative_state.state_grid #fixme: get current state_grid
                    self.latest_deviation_solution = partial_solutionizer.find_partial_solution_simple()
                    # todo: check which layouts are now obsolete and mark obsolete parts
                    difference = self.latest_deviation_solution.layouts.difference(self.tentative_state.aimed_solution.layouts)
                    for trail in difference:
                        obsolete_layout = self.tentative_state.construction_layouts.get(trail)
                        obsolete_fittings = obsolete_layout.fit_set
                        obsolete_attachments = obsolete_layout.att_set
                        obsolete_pipes = obsolete_layout.pipe_set

                    obsolete_parts = {"fittings": obsolete_fittings, "attachments": obsolete_attachments,
                                      "pipes": obsolete_pipes}

                    #todo: for rendering: visualization code should keep a reference of rendered objects pointing to the pos






        info_dict = {"current_layout": current_layout, "layout_changed": layout_changed,
                     "deviation_code": deviation_code, "pipe_id" : pipe_id, "layout_state": current_layout_state,
                     "deviation":deviation, "obsolete_parts": obsolete_parts}
        return info_dict

    def set_completion_state(self, current_layout, construction_layouts, removal):
        # check completion state
        neighboring_layouts = get_neighboring_layouts(current_layout, self.tentative_state.aimed_solution.layouts)
        neighboring_layouts.add(current_layout)

        for layout in neighboring_layouts:
            layout_state = construction_layouts[layout]
            if not layout_state.completed:
                if len(layout_state.fit_set) >= 2 and layout_state.pipe_set and layout_state.att_set:
                    # layout was completed
                    layout_state.completed = True
                    self.tentative_state.fc_set.add((current_layout[0], current_layout[-1]))
            else:
                if removal:
                    layout_state.completed = False
                    self.tentative_state.fc_set.remove((current_layout[0], current_layout[-1]))

    def determine_robot_commands(self, worker_event: tuple[Optional[int], Pos], info_dict:dict) -> tuple[list, list]:
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

        if layout_changed and event_code != 4: # last check is redundant
            if self.picking_robot_carries_part_id:
                picking_robot_commands.extend([-1,3,4])
            else:
                # robot doesn't carry part
                picking_robot_commands.extend([ (1, next_part_id), 3, 4])

        if event_code == 4:
            if next_part_id:
                picking_robot_commands.extend([(1, next_part_id), 3, 4])

        # make fastening robot commands
        if event_code == 3:
            fastening_robot_commands.append((1,event_pos))

        if obsolete_parts["attachments"]:
            fastening_robot_commands.append((-1,obsolete_parts["attachments"]))

        return fastening_robot_commands, picking_robot_commands

    def deviation_event(self, worker_event, tentative_state: State, debug_grid: Optional[np.ndarray]) -> dict:

        """Checks for deviating layouts on captured motion events and updates tentative state."""



        # todo: error handling:
        #   Possible errors:
        #   - motion event on confirmed occupied spot (check state_grid)
        #  todo: consider adding options to disable certain conditions (for testing)

        event_code = worker_event[0]
        event_pos = worker_event[1]

        if event_code == 1:

            for pos in self.latest_state.deviated_motion_pos_fitting:

                # check if conditions for a deviation event are met

                fit_diff = path_math.get_length_same_axis(pos,
                                                          event_pos)  # distance between fittings
                pipe_id = fit_diff-2 # length of needed part


                fittings_in_proximity = fit_diff in self.latest_state.part_stock.keys()  # fittings are connectable by available parts


                if not fittings_in_proximity:
                    continue

                fit_dir = get_direction(diff_pos(event_pos, pos))

                attachment_is_between = False

                att_set = set()
                pipe_set = set()

                for att_pos in tentative_state.deviated_motion_pos_attachment:
                    att_dir = get_direction(diff_pos(event_pos, att_pos))
                    if att_dir == fit_dir:
                        att_set.add(att_pos)

                if not att_set:
                    # no attachments in between
                    continue

                for pipe_pos in tentative_state.deviated_motion_pos_attachment:
                    pipe_dir = get_direction(diff_pos(event_pos, pipe_pos))
                    if pipe_dir == fit_dir:
                        pipe_set.add(pipe_pos)

                if not pipe_set:
                    # no pipes in between
                    continue


                # check if necessary parts have been picked
                if 0 in tentative_state.picked_parts and pipe_id in tentative_state.picked_parts:
                    # necessary parts have not been picked!
                    continue

                print("Deviation confirmed")

                fit_tup = (pos, event_pos)
                #get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, event_pos, pos)
                deviation_trail = get_deviation_trail(length=fit_diff, direction=fit_dir, fit_pos=fit_tup)
                deviation_state = get_deviation_state(length=fit_diff, att_set=att_set, pipe_set=pipe_set, fit_tup=fit_tup)

                tentative_state.fc_set.add(fit_tup)

                return {deviation_trail:deviation_state}
            # just update fitting pos
            tentative_state.deviated_motion_pos_fitting.add(event_pos)


    def update_latest_state(self, old_state, new_state):
        self.previous_states.insert(0, old_state)

        self.latest_state = new_state

    def return_to_previous_state(self):
        """Returns to the last valid state"""
        if self.previous_states:
            self.latest_state = self.previous_states.pop(0)
        else:
            print("There is no last state to return to!")





