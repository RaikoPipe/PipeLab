from vpython import vector, color
import numpy as np
import rendering
from data_class.State import State
from data_class.LayoutState import LayoutState
from path_finding.search_algorithm import find_path
from grid import grid_functions
from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
from data_class.Weights import Weights
from copy import deepcopy, copy
from path_finding.restriction_functions import get_worst_move_cost
from path_finding.path_utilities import get_direction
from path_finding.path_math import diff_pos
from path_finding.search_algorithm import find_path
from typing import Optional
import event_interpreting
from path_finding import path_math, partial_solutionizer

from path_finding.common_types import *
from utilities import *

standard_weights = Weights(1, 1, 1)
standard_algorithm = "mcsa*"

message_dict = {1:"fitting", 2:"pipe", 3:"attachment"}

# create new data type list[tuple] with tuple = (pos, part_id)

def get_current_part_stock(part_stock: dict, parts_used: list) -> dict:
    """Returns the part stock sans the parts already used."""
    for part_id in parts_used:
        part_stock[part_id] -= 1

    return part_stock


def get_new_solution(path_problem, weights):
    return find_path(path_problem=path_problem, weights=weights, algorithm=standard_algorithm)


example_motion_dict = {1: (1, 1)}  # considering motion capture speed, will probably never be bigger than 1

def check_solution_stack(completed_solutions_stack: dict, current_path_problem: PathProblem) -> Optional[Solution]:
    #todo: move to search_algorithm.py
    for path_problem in completed_solutions_stack:
        if path_problem == current_path_problem:
            return completed_solutions_stack[path_problem]
        else:
            return None


def get_current_path(state_grid, part_stock):
    """Returns the current paths from a state_grid"""
    trail_list = event_interpreting.get_trails_from_state_grid(state_grid=state_grid, searched_state=2)
    path_list = []
    for trail in trail_list:
        path = event_interpreting.get_path_from_trail(trail)
        path, _ = event_interpreting.correct_path_start(path, part_stock)
        path_list.append(path)
    return path_list


def get_updated_motion_dict(new_pos, motion_dict):
    for pos, event in new_pos.items():
        if pos == 0 and pos in motion_dict:
            motion_dict.pop(pos)
        else:
            motion_dict[pos] = event

    return motion_dict


def deviated_from_path(current_state: State, optimal_solution: Solution):

    for connection in current_state.fc_set:
        if connection not in optimal_solution.fc_set:
            return True
    else:
        return False

def get_updated_state_on_construction_event(tentative_state : State, fit_diff, fit_dir, new_pos, pos) -> State:
    # create new state

    # update fitting_connections
    tentative_state.connection_count[new_pos] = 1
    tentative_state.connection_count[pos] += 1

    # update fitting_pos
    tentative_state.motion_fitting_pos = new_pos

    # update part stock
    tentative_state.part_stock[fit_diff] -= 1  # reduce pipe stock
    tentative_state.part_stock[0] -= 1  # reduce fitting stock

    # if any of the following pos is None, there was an error
    pipe_start_pos = None
    pipe_end_pos = None

    # update state grid
    construction_trail = []

    #set construction_parts fittings
    construction_parts = {pos: 0, new_pos: 0}

    for i in range(fit_diff + 1):
        state_pos = (pos[0] + fit_dir[0] * i, pos[1] + fit_dir[1] * i)
        tentative_state.state_grid[state_pos] = 2
        construction_trail.append(state_pos)

        # save info about pipes for later
        if i == 1:
            pipe_start_pos = state_pos
        elif i == fit_diff:
            pipe_end_pos = state_pos
        elif i != 0 and i != fit_diff+1:
            construction_parts[state_pos] = fit_diff
            # add to construction_parts


    if pipe_start_pos is None or pipe_end_pos is None:
        print("Error Code XX")

    # reduce picked parts counter
    tentative_state.picked_parts[0] -= 1
    tentative_state.picked_parts[fit_diff] -= 1

    # add to construction trails
    tentative_state.construction_layouts[construction_trail] = 2
    #todo: use construction trail to remove all motion sets

    # create new layout

    new_construction_path = DefinitePath()

    new_construction_path.append((new_pos, None))
    new_construction_path.append(pipe_start_pos, 0)
    new_construction_path.append(pipe_end_pos, fit_diff)
    new_construction_path.append(pos, 0)

    new_connection =  (new_pos, pos)

    #add to layouts
    tentative_state.layouts.add(new_construction_path)
    #add to connections
    tentative_state.fc_set.add(new_connection)
    #add to construction_parts
    tentative_state.construction_parts.update(construction_parts)
    if new_connection in tentative_state.removed_fc_set:
        tentative_state.removed_fc_set.remove(new_connection)

    return tentative_state


class ProcessPlanner:
    """Acts as an interface for handling events. Keeps track of the building process and provides new solutions on events. Returns instructions."""
    #todo: Tasks
    #   - Use robot events and worker events to plan next step, give robot commands
    def __init__(self, initial_path_problem: PathProblem, initial_state: State,
                 optimization_weights: Weights = standard_weights,
                 algorithm: str = standard_algorithm):

        self.completed_solutions_stack = {}  # PathProblem: Solution #fixme: should be in optimizer

        self._initial_path_problem = initial_path_problem  # original path problem
        self.optimal_solution = find_path(self._initial_path_problem)  # optimal solution for the initial path problem
        self.latest_deviation_solution = None

        self.previous_states = []  # contains all previous valid states

        self.latest_state = initial_state  # latest valid state
        self.latest_state.aimed_solution = self.optimal_solution
        # add layouts of aimed solution if not already there
        for layouts in self.latest_state.aimed_solution.layouts:
            if self.latest_state.construction_layouts.get(layouts) is None:
                #todo: add layout states from optimal solution
        self.tentative_state = deepcopy(initial_state)
        # Build task is complete, if there is a DefinitePath in definite_paths of latest_state that matches a DefinitePath of an optimal solution
        # If there are DefinitePaths left over, these can be highlighted to remove leftover parts from the build process
        self.is_optimal = True  # if path of current state is on optimal solution
        self.tentative_state.latest_layout = self.optimal_solution.layouts[0] # layout that is currently being built. Initial layout is at start.


        self.weights = optimization_weights
        self.algorithm = algorithm

        self.motion_dict = {}

        self.debug_motion_grid = self._initial_path_problem.state_grid

        # picking robot state
        self.picking_robot_carries_part_id = None

    """Biggest Problem of using motions to detect constructions: If an error occurs, there is no way to reassess the current process state.
    There should be a redundant factor (like a camera used for CV) that can capture the current construction layout.
     --> Motion Detection needs extremely high accuracy in order to be reliable."""

    # def handle_new_input(self, worker_event: tuple[Optional[int], Pos], pick_events: list) -> bool:
    #     """evaluates worker event (and event pos on the mounting wall) to determine the current construction layout.
    #     :returns deviation occurrence"""
    #     # fixme: deepcopy might be slow, consider alternative
    #
    #
    #
    #     deviation_occurred = False
    #
    #     if pick_events:
    #         for part_id in pick_events:
    #             self.tentative_state.picked_parts[part_id] += 1
    #
    #     event_code = worker_event[0]
    #     event_pos = worker_event[1]
    #
    #     if worker_event[0]:
    #
    #         if event_code == 2:
    #             self.tentative_state.motion_pipe_pos.add(event_pos)
    #         elif event_code == 3:
    #             self.tentative_state.motion_attachment_pos.add(event_pos)
    #
    #         #todo: check for removals, update tentative state
    #
    #         if self.deviation_event(motion_events=worker_event, tentative_state=self.tentative_state):
    #             if self.tentative_state.deviated_from_opt_sol:
    #
    #                 if not deviated_from_path(current_state=self.tentative_state, optimal_solution=self.optimal_solution):
    #                     self.tentative_state.deviated_from_opt_sol = False
    #                     # todo: notify worker?
    #                 #todo: check if deviated from latest deviation solution
    #                 elif deviated_from_path(current_state=self.tentative_state, optimal_solution=self.latest_deviation_solution):
    #                     deviation_occurred = True
    #                     # todo: create new partial solution
    #                     self.latest_deviation_solution = partial_solutionizer.find_partial_solution_simple(self.tentative_state.layouts,
    #                                                                                                        self.tentative_state.state_grid,
    #                                                                                                        self._initial_path_problem)
    #                     self.tentative_state.aimed_solution = self.latest_deviation_solution
    #             else:
    #                 if deviated_from_path(current_state=self.tentative_state, optimal_solution=self.optimal_solution):
    #                     deviation_occurred = True
    #                     self.tentative_state.deviated_from_opt_sol = True
    #                     self.latest_deviation_solution = partial_solutionizer.find_partial_solution_simple(
    #                         self.tentative_state.layouts,
    #                         self.tentative_state.state_grid,
    #                         self._initial_path_problem)
    #
    #         if self.deconstruction_event
    #
    #
    #     #todo: recognize pick events
    #     # detect if part was picked by worker: if picking event occurs outside robot state pick part
    #
    #
    #     if pick_events or worker_event:
    #         self.update_latest_state(old_state=self.latest_state, new_state=self.tentative_state)
    #
    #     return deviation_occurred



    def make_registration_message(self, event_pos:Pos, event_code:int):
        object_name = message_dict[event_code]
        print(str.format(f"Registered {object_name} at Position {event_pos}"))

    def make_special_message(self, message:str, event_pos):
        print(str.format(f"Position {event_pos}: {message} "))


    def make_error_message(self, event_pos, additional_message:str):
        print(str.format(f"Possible detection error at Position {event_pos}: {additional_message}"))

        # todo: use removal list to highlight objects that need to be removed
    def new_construction_check(self, worker_event: tuple[Optional[int], Pos]):


        worker_event_pos = worker_event[1]
        worker_event_code =worker_event[0]

        detection_error = False
        process_deviation_inside_aimed_sol = False
        process_deviation_outside_aimed_sol = False
        error_code = None
        """error codes
        -1: detection error: part was placed on an occupied spot
        0: process deviation: unnecessary part inside solution
        1: process deviation: part was placed incorrectly inside solution
        1: process deviation: part was placed outside solution
        2: process deviation: part was placed, but never picked prior"""


        if worker_event_code == 4:
            #todo: add rendering instruction: highlight placement positions
            pass

        #check if deviation event occurred
        #todo: don't forget to clean up all the other detected motions + finished layouts outside!
        if worker_event_pos not in self.tentative_state.aimed_solution.total_definite_trail.keys():
            # motion event occurred outside
            process_deviation_outside_aimed_sol = True
            if worker_event_code == 3:
                self.tentative_state.motion_attachment_pos.add(worker_event_pos)
            elif worker_event_code == 2:
                self.tentative_state.motion_pipe_pos.add(worker_event_pos)
            elif worker_event_code == 1:
                # check for deviation events
                deviation = self.deviation_event(worker_event, self.tentative_state, None)
            elif worker_event_code == 0:
                # todo: check for deconstruction events
                pass

        else:

            # get information about the current layout
            current_layout = None
            for trail in self.tentative_state.construction_layouts.keys():
                if worker_event_pos in trail:
                    current_layout = trail

            layout_changed = current_layout != self.tentative_state.latest_layout

            current_layout_state = self.tentative_state.construction_layouts[current_layout]

            #todo: handle to more special situations in 3: placement either in correct_fitting_pos or
            # todo: if pipe was placed somewhere inside a layout, its fine.
            
            if worker_event_code == 3:
                self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code)
                if not current_layout_state.attachment_pos:
                    # successful placement
                    current_layout_state.attachment_pos.add(worker_event_pos)
                else:
                    # Unneeded attachment detected!
                    if not worker_event_pos in current_layout_state.attachment_pos:
                        # attachment is unnecessary
                        process_deviation_inside_aimed_sol = True
                        self.make_special_message(message="Unnecessary attachment detected at: ", event_pos=worker_event_pos)
                        self.tentative_state.remove_parts[worker_event_pos] = -1

                    else:
                        # error: layout already has an attachment at this position!
                        detection_error = True
                        self.make_error_message(event_pos=worker_event_pos, additional_message="Already has attachment at this position!")
                        self.tentative_state.error_dict[worker_event_pos] = -1

            elif worker_event_code == 2:
                self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code)
                if not current_layout_state.pipe_pos:
                    # check if part was actually picked
                    if self.tentative_state.picked_parts[current_layout.pipe_id] > 0:
                        # successful placement
                        self.tentative_state.picked_parts[current_layout.pipe_id] -= 1
                        current_layout_state.pipe_pos.add(worker_event_pos)
                    else:
                        # part was not picked!
                        detection_error = True
                        self.make_error_message(event_pos=worker_event_pos,
                                                additional_message=
                                                str.format(f"Part with id {current_layout_state.pipe_pos} "
                                                           f"was placed, but not picked!"))
                        self.tentative_state.error_dict[worker_event_pos] = current_layout_state.pipe_id
                else:
                    # layout already has pipe!
                    detection_error = True
                    self.make_error_message(event_pos=worker_event_pos, additional_message="Layout already has pipe!")
                    self.tentative_state.error_dict[worker_event_pos] = current_layout_state.pipe_id

            elif worker_event_code == 1:
                self.make_registration_message(event_pos=worker_event_pos, event_code=worker_event_code)
                if current_layout_state.fitting_pos < 2 and worker_event_pos in current_layout_state.correct_fitting_pos:
                    if self.tentative_state.picked_parts[0] > 0:
                        # successful placement
                        self.tentative_state.picked_parts[0] -= 1
                        current_layout_state.fitting_pos.add(worker_event_pos)


                    else:
                        # part was not picked!
                        process = True
                        self.make_error_message(event_pos=worker_event_pos,
                                                additional_message=
                                                str.format(f"Part with id {0} "
                                                           f"was placed, but not picked!"))



                else:
                    # Unneeded fitting detected!
                    if not worker_event_pos in current_layout_state.attachment_pos:
                        # fitting is unnecessary
                        process_deviation_inside_aimed_sol = True
                        self.make_special_message(message="Unnecessary fitting detected at: ", event_pos=worker_event_pos)
                        self.tentative_state.remove_parts[worker_event_pos] = 0
                    else:
                        # error: layout already has fittings at this position!
                        detection_error = True
                        self.make_error_message(event_pos=worker_event_pos, additional_message="Already has fitting at this position!")
            elif worker_event_code == 0:
                # todo: repurpose deconstruction events
                # on pipe removal: use pos of attachment to command fasten robot to unscrew attachment

            #update layout states of construction layouts
            self.tentative_state.construction_layouts[current_layout] = current_layout_state

            # todo: command to picking robot: retrieve part (return current one if necessary)
            # todo: command to fastening robot: fasten attachment
        return process_deviation_outside_aimed_sol, process_deviation_inside_aimed_sol, detection_error

    def determine_next_step(self, picking_robot_state: list, fastening_robot_state: list,
                            worker_event: tuple[Optional[int], Pos], deviation_occurred: bool):
        """Evaluates the current process state and issues robot commands"""

        #fixme: repurpose this function to only issue commands

        event_code = worker_event[0]
        event_pos = worker_event[1]

        retrieve_part_id = None

        fastening_robot_commands = []
        picking_robot_commands = []

        #todo: separately handle event code 4
        # todo: command to picking robot: pick next part
        if event_code == 4:
            return




        if deviation_occurred:
            # reevaluate next step based on deviation_solution, stop current action of robot

        elif event_pos not in self.tentative_state.optimal_solution.total_definite_trail.keys():



        #todo: implement internal counter: if attachment motion occurred twice, then change behaviour of picking robot


        #todo: check latest worker event, save last process step
        elif not deviation_occurred:





            if event_pos not in self.tentative_state.latest_layout:
                # motion was detected outside current solution layout

            elif event_code == 3:
                # signifies the start of a new construction
                if current_layout_state.attachment_pos:
                    # Unneeded attachment detected!
                    if event_pos in current_layout_state.attachment_pos:
                        print("Detection Error: Position " + str(event_pos) +" already has an attachment!")
                    else:
                        # todo: add rendering instruction: highlight unneeded attachment
                else: # no attachment here
                    #update
                    current_layout_state.attachment_pos.add(event_pos)


                retrieve_part_id = self.optimal_solution.total_definite_trail[event_pos]

                # todo: command to picking robot: retrieve part (return current one if necessary)
                # todo: command to fastening robot: fasten attachment

            elif event_code == 2:
                # todo: command to fasten robot: fasten at pos...
            elif event_code == 1:
                # todo: command to fasten robot: fasten fitting
            elif event_code == 0:
                # todo: offer return of part
        # todo: save latest commands




    def deconstruction_event(self, motion_events: dict, tentative_state: State, debug_grid: Optional[np.ndarray]) -> bool:

        """Checks for deconstruction events on captured motion events and updates tentative state."""

        for new_pos, event in motion_events.items():

            if event == 0:
                if new_pos in tentative_state.motion_fitting_pos:

                    for fit_set in tentative_state.fc_set:
                        if new_pos in fit_set:
                            tentative_state.fc_set.remove(fit_set)
                            tentative_state.removed_fc_set.add(fit_set)
                            tentative_state.motion_fitting_pos.remove(new_pos)
                            break

                else:
                    for trail in tentative_state.construction_layouts.keys():
                        if new_pos in trail:
                            if tentative_state.construction_layouts[trail] > 0:
                                tentative_state.construction_layouts[trail] -= 1
                        if tentative_state.construction_layouts[trail] == 0:
                            # construction has been fully removed
                            tentative_state.removed_fc_set.remove({trail[0], trail[-1]})

                print("Deconstruction confirmed")

                #get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, new_pos, pos)

                return True


    def deviation_event(self, worker_event, tentative_state: State, debug_grid: Optional[np.ndarray]) -> bool:

        """Checks for construction events on captured motion events and updates tentative state."""



        # todo: error handling:
        #   Possible errors:
        #   - motion event on confirmed occupied spot (check state_grid)
        #  todo: consider adding options to disable certain conditions (for testing)

        event_code = worker_event[0]
        event_pos = worker_event[1]

        if event_code == 1:

            for pos in self.latest_state.fittings_pos:

                # check if conditions for a construction event are met

                fit_diff = path_math.get_length_same_axis(pos,
                                                          new_pos)  # distance between fittings -> length of needed part

                # todo: also check if part was actually picked
                fittings_in_proximity = fit_diff in self.latest_state.part_stock.keys()  # fittings are connectable by available parts


                if not fittings_in_proximity:
                    continue

                fit_dir = get_direction(diff_pos(new_pos, pos))

                pipe_attachment_is_between = False

                for att_pos in tentative_state.motion_attachment_pos:
                    if att_pos not in tentative_state.motion_pipe_pos:  # if no pipe has been attached
                        continue
                    att_dir = get_direction(diff_pos(new_pos, att_pos))
                    pipe_attachment_is_between = att_dir == fit_dir  # An attachment is in between considered fittings

                if not pipe_attachment_is_between:
                    continue


                # check if necessary parts have been picked
                if 0 in tentative_state.picked_parts and fit_diff in tentative_state.picked_parts:
                    continue

                print("Construction confirmed")

                get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, new_pos, pos)

                return True
            # just update fitting pos
            tentative_state.motion_fitting_pos.add(new_pos)


    def update_latest_state(self, old_state, new_state):
        self.previous_states.insert(0, old_state)

        self.latest_state = new_state

    def return_to_previous_state(self):
        """Returns to the last valid state"""
        if self.previous_states:
            self.latest_state = self.previous_states.pop(0)
        else:
            print("There is no last state to return to!")

    # def event_received_robot_state(self, state_id):
    #     # todo: determine next step
    #     print("not implemented")
    #
    # def event_captured_state_grid(self, captured_state_grid):
    #     # todo: interpret new data, exec actions as needed, update latest path problem, update current_layout_solution
    #     if event_interpreting.grid_changed(latest_state_grid=self.latest_state.state_grid,
    #                                        captured_state_grid=captured_state_grid):
    #
    #         event = event_interpreting.check_action_event(picked_parts=self.picked_parts,
    #                                                       placed_parts=self.placed_parts,
    #                                                       latest_state_grid=self.latest_path_problem.state_grid,
    #                                                       captured_state_grid=captured_state_grid)
    #         # todo: on event 1: check if path of current state overlaps with optimal solution ->
    #         if event["code"] == -1:
    #             # recalculate path of current state
    #             trail_list = event_interpreting.get_trails_from_state_grid(state_grid=captured_state_grid,
    #                                                                        searched_state=2)
    #             for trail in trail_list:
    #                 path = event_interpreting.get_path_from_trail(trail)
    #                 definite_path = event_interpreting.get_definite_path_from_path(path=path,
    #                                                                                part_stock=self._initial_path_problem.part_stock)
    #
    #             pass
    #         elif event["code"] == 1:
    #             # todo: check where part was removed, change latest_state.definite_path, check if we are back on optimal solution
    #
    #             pass
    #         elif event["code"] == 2:
    #             # todo: check where part has been placed and if it is adjacent to a current layout and valid -> expand this path
    #             # todo: check if paths have been connected: fuse into one path
    #             # todo: check if current path overlaps with optimal solution
    #             pass
    #         else:
    #             print("Unknown Error occurred!")
    #
    # def update_current_state(self, state_grid: np.ndarray, path: DefinitePath):
    #     self.latest_state.state_grid = state_grid
    #     self.latest_state.definite_path = path
    #
    # def event_part_id_picked(self, part_id_picked):
    #     self.picked_parts.append(part_id_picked)
    #     # todo: highlight placement options in visualization



