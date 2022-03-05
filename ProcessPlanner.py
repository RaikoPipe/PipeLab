from vpython import vector, color
import numpy as np
import rendering
from data_class import State
from path_finding.search_algorithm import find_path
from grid import grid_functions
from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
from data_class.Weights import Weights
from copy import deepcopy, copy
from path_finding.restriction_functions import get_worst_move_cost
from path_finding.path_utilities import get_direction
from path_finding.p_math import diff_pos
from path_finding.search_algorithm import find_path
from typing import Optional
import event_interpreting
from path_finding import p_math, partial_solutionizer

from path_finding.common_types import *
from utilities import *

# todo: finish later

standard_weights = Weights(1, 1, 1)
standard_algorithm = "mcsa*"


# todo: create comparison function for comparing optimal solution to current solution
# create new data type list[tuple] with tuple = (pos, part_id)

def event_part_placed():
    """"""


def event_part_removed():
    """"""
    # todo: check where part was removed and if path was split or just reduced


def get_current_part_stock(part_stock: dict, parts_used: list) -> dict:
    """Returns the part stock sans the parts already used."""
    for part_id in parts_used:
        part_stock[part_id] -= 1

    return part_stock


def get_new_solution(path_problem, weights):
    return find_path(path_problem=path_problem, weights=weights, algorithm=standard_algorithm)


example_motion_dict = {1: (1, 1)}  # considering motion capture speed, will probably never be bigger than 1


# todo: it is assumed that the sensor detects changes on the grid and the layout (current path and currently used parts)

def check_solution_stack(completed_solutions_stack: dict, current_path_problem: PathProblem) -> Optional[Solution]:
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

    #todo: redo. Use Connections this time
    # put pos, part_id into separate sets

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
    for i in range(fit_diff + 1):
        state_pos = (pos[0] + fit_dir[0] * i, pos[1] + fit_dir[1] * i)
        tentative_state.state_grid[state_pos] = 2
        construction_trail.append(state_pos)

        # save info about pipes for later
        if i == 1:
            pipe_start_pos = state_pos
        elif i == fit_diff:
            pipe_end_pos = state_pos
    if pipe_start_pos is None or pipe_end_pos is None:
        print("Error Code XX")

    # reduce picked parts counter
    tentative_state.picked_parts[0] -= 1
    tentative_state.picked_parts[fit_diff] -= 1

    # add to construction trails
    tentative_state.construction_trails[construction_trail] = 2
    #todo: use construction trail to remove all motion events

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
        # Build task is complete, if there is a DefinitePath in definite_paths of latest_state that matches a DefinitePath of an optimal solution
        # If there are DefinitePaths left over, these can be highlighted to remove leftover parts from the build process
        self.is_optimal = True  # if path of current state is on optimal solution

        self.weights = optimization_weights
        self.algorithm = algorithm

        self.motion_dict = {}

        self.debug_motion_grid = self._initial_path_problem.state_grid

    """Biggest Problem of using motions to detect constructions: If an error occurs, there is no way to reassess the current process state.
    There should be a redundant factor (like a camera used for CV) that can capture the current construction layout.
     --> Motion Detection needs extremely high accuracy in order to be reliable."""

    def handle_new_input(self, motion_events: dict, pick_events: list, robot_events: list, worker_events: list):
        # fixme: deepcopy might be slow, consider alternative
        # fixme: current assumption: only one motion event per pos; fix: motion event as list, order of list is event order
        # fixme: attachment has two states: 1: attachment attached and 2: attachment closed
        #   -> pipe attachment motions always happen at the same pos!
        """
            motion event codes
            0 -> removal
            1 -> fitting placed
            2 -> pipe placed
            3 -> attachment placed
        """

        tentative_state = deepcopy(self.latest_state)
        if pick_events:
            for part_id in pick_events:
                tentative_state.picked_parts[part_id] += 1
        if motion_events:

            for new_pos, event in motion_events.items():
                if event == 2:
                    tentative_state.motion_pipe_pos.add(new_pos)
                elif event == 3:
                    tentative_state.motion_attachment_pos.add(new_pos)

            #todo: check for removals, update tentative state

            if self.construction_event(motion_events=motion_events, tentative_state=tentative_state):
                if tentative_state.deviated:
                    #todo: check if back on optimal solution
                    if not deviated_from_path(current_state=tentative_state, optimal_solution=self.optimal_solution):
                        tentative_state.deviated = False
                        # todo: notify worker?
                    #todo: check if deviated from latest deviation solution
                    elif deviated_from_path(current_state=tentative_state, optimal_solution=self.latest_deviation_solution):
                        # todo: create new partial solution
                        self.latest_deviation_solution = partial_solutionizer.find_partial_solution_simple(tentative_state.layouts,
                                                                                                           tentative_state.state_grid,
                                                                                                           self._initial_path_problem)
                else:
                    if deviated_from_path(current_state=tentative_state, optimal_solution=self.optimal_solution):
                        tentative_state.deviated = True
                        self.latest_deviation_solution = partial_solutionizer.find_partial_solution_simple(
                            tentative_state.layouts,
                            tentative_state.state_grid,
                            self._initial_path_problem)

            if self.deconstruction_event





        if pick_events or motion_events:
            self.update_latest_state(old_state=self.latest_state, new_state=tentative_state)

    def deconstruction_event(self, motion_events: dict, tentative_state: State, debug_grid: Optional[np.ndarray]) -> bool:

        """Checks for construction events on captured motion events and updates tentative state."""

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
                    for trail in tentative_state.construction_trails.keys():
                        if new_pos in trail:
                            if tentative_state.construction_trails[trail] > 0:
                                tentative_state.construction_trails[trail] -= 1
                        if tentative_state.construction_trails[trail] == 0:
                            # construction has been fully removed
                            tentative_state.removed_fc_set.remove({trail[0], trail[-1]})

                print("Deconstruction confirmed")

                #get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, new_pos, pos)

                return True


    def construction_event(self, motion_events: dict, tentative_state: State, debug_grid: Optional[np.ndarray]) -> bool:

        """Checks for construction events on captured motion events and updates tentative state."""



        # todo: error handling:
        #   Possible errors:
        #   - motion event on confirmed occupied spot (check state_grid)
        #  todo: consider adding options to disable certain conditions (for testing)

        for new_pos, event in motion_events.items():

            if event == 1:

                for pos in self.latest_state.fittings_pos:

                    # check if conditions for a construction event are met

                    fit_diff = p_math.get_length_same_axis(pos,
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



