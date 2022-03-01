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
from path_finding.restriction_functions import get_direction, get_worst_move_cost
from path_finding.p_math import diff_pos
from path_finding.search_algorithm import find_path
from typing import Optional
import event_interpreting
from path_finding import p_math

from path_finding.common_types import *

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


class ProcessPlanner:
    """Acts as an interface for handling events. Keeps track of the building process and provides new solutions on events. Returns instructions."""

    def __init__(self, initial_path_problem: PathProblem, initial_state: State,
                 optimization_weights: Weights = standard_weights,
                 algorithm: str = standard_algorithm):

        self.completed_solutions_stack = {}  # PathProblem: Solution

        self._initial_path_problem = initial_path_problem  # original path problem
        self.optimal_solution = find_path(self._initial_path_problem)  # optimal solution for the initial path problem

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

    def handle_new_input(self, motion_events: dict, pick_events: set, robot_events: int, worker_events: int):
        # fixme: deepcopy might be slow, consider alternative
        # fixme: current assumption: only one motion event per pos; fix: motion event as list, order of list is event order
        tentative_state = deepcopy(self.latest_state)
        if pick_events:
            for part_id in pick_events:
                tentative_state.picked_parts[part_id] += 1
        if motion_events:

            for new_pos, event in motion_events.items():
                if event == 2:
                    tentative_state.pipe_pos.insert(0, new_pos)
                elif event == 3:
                    tentative_state.attachment_pos.insert(0, new_pos)

            #todo: check for removals, update tentative state

            if self.construction_event_confirmed(motion_events=motion_events, tentative_state=tentative_state):

                # todo: check for deviations


        if pick_events or motion_events:
            self.update_latest_state(old_state=self.latest_state, new_state=tentative_state)



    def construction_event_confirmed(self, motion_events: dict, tentative_state: State, debug_grid: Optional[np.ndarray]) -> bool:

        """Checks for construction events on captured motion events and updates tentative state."""

        """
            position codes
            0 -> removal
            1 -> fitting placed
            2 -> pipe placed
            3 -> attachment placed

            fittings dict
            pos : number of connections
        """

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

                    fittings_in_proximity = fit_diff in self.latest_state.part_stock.keys()  # fittings are connectable by available parts

                    if not fittings_in_proximity:
                        continue

                    fit_dir = get_direction(diff_pos(new_pos, pos))

                    pipe_attachment_is_between = False

                    for att_pos in tentative_state.attachment_pos:
                        if att_pos not in tentative_state.pipe_pos:  # if no pipe has been attached
                            continue
                        att_dir = get_direction(diff_pos(new_pos, att_pos))
                        pipe_attachment_is_between = att_dir == fit_dir  # An attachment is in between considered fittings

                    if not pipe_attachment_is_between:
                        continue


                    # check if necessary parts have been picked
                    if 0 in tentative_state.picked_parts and fit_diff in tentative_state.picked_parts:
                        continue

                    print("Construction confirmed")

                    self.get_updated_state_on_construction_event(tentative_state, fit_diff, fit_dir, new_pos, pos)

                    return True


    def update_latest_state(self, old_state, new_state):
        self.previous_states.insert(0, old_state)

        self.latest_state = new_state

    def get_updated_state_on_construction_event(self, tentative_state : State, fit_diff, fit_dir, new_pos, pos) -> State:
        # create new state

        # update fitting_connections
        tentative_state.fitting_connections[new_pos] = 1
        tentative_state.fitting_connections[pos] += 1

        # update fitting_pos
        tentative_state.fitting_pos = new_pos

        # update part stock
        tentative_state.part_stock[fit_diff] -= 1  # reduce pipe stock
        tentative_state.part_stock[0] -= 1  # reduce fitting stock

        # if any of the following pos is None, there was an error
        pipe_start_pos = None
        pipe_end_pos = None

        # update state grid
        for i in range(fit_diff + 1):
            state_pos = (pos[0] + fit_dir[0] * i, pos[1] + fit_dir[1] * i)
            tentative_state.state_grid[state_pos] = 2

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

        # create new layout

        new_construction_layout = DefinitePath()

        new_construction_layout.append((new_pos, None))
        new_construction_layout.append(pipe_start_pos, 0)
        new_construction_layout.append(pipe_end_pos, fit_diff)
        new_construction_layout.append(pos, 0)

        #add to layouts
        tentative_state.add(new_construction_layout)

        return tentative_state

    def return_to_previous_state(self):
        """Returns to the last valid state"""
        if self.previous_states:
            self.latest_state = self.previous_states.pop(0)
        else:
            print("There is no last state to return to!")

    def event_received_robot_state(self, state_id):
        # todo: determine next step
        print("not implemented")

    def event_captured_state_grid(self, captured_state_grid):
        # todo: interpret new data, exec actions as needed, update latest path problem, update current_layout_solution
        if event_interpreting.grid_changed(latest_state_grid=self.latest_state.state_grid,
                                           captured_state_grid=captured_state_grid):

            event = event_interpreting.check_action_event(picked_parts=self.picked_parts,
                                                          placed_parts=self.placed_parts,
                                                          latest_state_grid=self.latest_path_problem.state_grid,
                                                          captured_state_grid=captured_state_grid)
            # todo: on event 1: check if path of current state overlaps with optimal solution ->
            if event["code"] == -1:
                # recalculate path of current state
                trail_list = event_interpreting.get_trails_from_state_grid(state_grid=captured_state_grid,
                                                                           searched_state=2)
                for trail in trail_list:
                    path = event_interpreting.get_path_from_trail(trail)
                    definite_path = event_interpreting.get_definite_path_from_path(path=path,
                                                                                   part_stock=self._initial_path_problem.part_stock)

                pass
            elif event["code"] == 1:
                # todo: check where part was removed, change latest_state.definite_path, check if we are back on optimal solution

                pass
            elif event["code"] == 2:
                # todo: check where part has been placed and if it is adjacent to a current layout and valid -> expand this path
                # todo: check if paths have been connected: fuse into one path
                # todo: check if current path overlaps with optimal solution
                pass
            else:
                print("Unknown Error occurred!")

    def update_current_state(self, state_grid: np.ndarray, path: DefinitePath):
        self.latest_state.state_grid = state_grid
        self.latest_state.definite_path = path

    def event_part_id_picked(self, part_id_picked):
        self.picked_parts.append(part_id_picked)
        # todo: highlight placement options in visualization

    # def grid_check(self, captured_state_grid: np.ndarray, parts_used : list, path: Path) -> Optional[Solution]:
    #     """Checks for changes in the captured grid and returns a solution on change (if solvable)."""
    #     if grid_changed(captured_state_grid, self.latest_path_problem.state_grid):
    #         self.latest_path_problem = self.get_new_path_problem(state_grid=captured_state_grid, parts_used=parts_used, path=path)
    #         # todo: check if problem is solved/layout completed (last entry in path = goal)
    #         #  -> no new solution needed
    #         self.latest_state = self.get_current_layout_solution(captured_state_grid, parts_used, path)
    #
    #         solution = check_solution_stack(self.completed_solutions_stack, self.latest_path_problem)
    #         if solution is None:
    #             solution = get_new_solution(path_problem=self.latest_path_problem, weights=self.weights)
    #         return solution
    #
    #     return None # grid has not changed, nothing to do.

    # def get_new_path_problem(self, state_grid: np.ndarray, parts_used:list, path:list) -> PathProblem:
    #     """Makes a copy of the initial path problem and adjusts it according to parameters. Returns the adjusted copy."""
    #
    #     new_path_problem = deepcopy(self._initial_path_problem) # make a copy of the original path problem
    #
    #     current_pipe_stock = get_current_part_stock(copy(self._initial_path_problem.part_stock), parts_used)
    #     new_path_problem.part_stock = current_pipe_stock
    #     new_path_problem.state_grid = state_grid
    #     new_path_problem.start_node = path[-1] # last entry is new start pos
    #     new_path_problem.start_direction = get_direction(diff_pos(path[-2], path[-1])) # direction from second last entry to last entry is start direction
    #     # goal parameters stay the same
    #
    #     return new_path_problem
    #
    # def get_current_layout_solution(self, captured_state_grid: np.ndarray, parts_used: list, path: list):
    #     new_layout_solution = Solution(path=path, algorithm=None, parts=parts_used, path_problem=self._initial_path_problem,
    #                                    problem_solved=False, score = None, solution_grid=captured_state_grid)
    #     return new_layout_solution
    #
    # def get_invalid_solutions(self, remove:bool=True) -> list:
    #     """Gets invalid solutions and returns them. Optionally also removes them. Should be called after modifying the
    #      path problem."""
    #
    #     #todo: code function
