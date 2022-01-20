from vpython import vector, color
import numpy as np
import rendering
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

from path_finding.common_types import *

# todo: finish later

standard_weights = Weights(1,1,1)
standard_algorithm = "mcsa*"


#todo: create comparison function for comparing optimal solution to current solution
# create new data type list[tuple] with tuple = (pos, part_id)

def event_part_placed():
    """"""
    # todo: check where part has been placed and if it is adjacent to a current layout and valid -> expand this path
    # todo: check if paths have been connected: fuse into one path
    # todo: check if current path overlaps with optimal solution

def event_part_removed():
    """"""
    #todo: check where part was removed and if path was split or just reduced



def get_current_part_stock(part_stock: dict, parts_used:list) -> dict:
    """Returns the part stock sans the parts already used."""
    for part_id in parts_used:
        part_stock[part_id] -= 1

    return part_stock

def get_new_solution(path_problem, weights):
    return find_path(path_problem=path_problem, weights=weights, algorithm=standard_algorithm)

#todo: it is assumed that the sensor detects changes on the grid and the layout (current path and currently used parts)

def check_solution_stack(completed_solutions_stack: dict, current_path_problem: PathProblem) -> Optional[Solution]:
    for path_problem in completed_solutions_stack:
        if path_problem == current_path_problem:
            return completed_solutions_stack[path_problem]
        else: return None



class ProcessPlanner:
    """Acts as an interface for handling events. Keeps track of the building process and provides new solutions on events. Returns instructions."""
    def __init__(self, initial_path_problem: PathProblem, current_layout_solution: Optional[Solution], optimization_weights: Weights = standard_weights,
                 algorithm : str = standard_algorithm):

        self.completed_solutions_stack = {} # PathProblem: Solution

        self._initial_path_problem = initial_path_problem # original path problem
        self.optimal_solution = find_path(self._initial_path_problem)

        self.previous_states = []

        self.latest_state = current_layout_solution

        self.weights = optimization_weights
        self.algorithm = algorithm

        self.picked_parts = []
        self.placed_parts = []

    #todo: received_events must be handled at the same time (like routes on a flask server)

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
        #todo: interpret new data, exec actions as needed, update latest path problem, update current_layout_solution
        if event_interpreting.grid_changed(latest_state_grid=self.latest_state.state_grid, captured_state_grid=captured_state_grid):

            event = event_interpreting.check_action_event(picked_parts=self.picked_parts, placed_parts=self.placed_parts, latest_state_grid=self.latest_path_problem.state_grid,
                                                          captured_state_grid=captured_state_grid)
            if event["code"] == -2:
                print("Unkown Error occurred!")
            elif event["code"] == -1:
                #todo: recalculate current path
            elif event["code"] == 1:
                # todo:

            self.latest_path_problem.state_grid = captured_state_grid



    def event_part_id_picked(self, part_id_picked):
        self.picked_parts.append(part_id_picked)
        # todo: highlight placement options in visualization







    def grid_check(self, captured_state_grid: np.ndarray, parts_used : list, path: Path) -> Optional[Solution]:
        """Checks for changes in the captured grid and returns a solution on change (if solvable)."""
        if grid_changed(captured_state_grid, self.latest_path_problem.state_grid):
            self.latest_path_problem = self.get_new_path_problem(state_grid=captured_state_grid, parts_used=parts_used, path=path)
            # todo: check if problem is solved/layout completed (last entry in path = goal)
            #  -> no new solution needed
            self.latest_state = self.get_current_layout_solution(captured_state_grid, parts_used, path)

            solution = check_solution_stack(self.completed_solutions_stack, self.latest_path_problem)
            if solution is None:
                solution = get_new_solution(path_problem=self.latest_path_problem, weights=self.weights)
            return solution

        return None # grid has not changed, nothing to do.

    def get_new_path_problem(self, state_grid: np.ndarray, parts_used:list, path:list) -> PathProblem:
        """Makes a copy of the initial path problem and adjusts it according to parameters. Returns the adjusted copy."""

        new_path_problem = deepcopy(self._initial_path_problem) # make a copy of the original path problem

        current_pipe_stock = get_current_part_stock(copy(self._initial_path_problem.part_stock), parts_used)
        new_path_problem.part_stock = current_pipe_stock
        new_path_problem.state_grid = state_grid
        new_path_problem.start_node = path[-1] # last entry is new start pos
        new_path_problem.start_direction = get_direction(diff_pos(path[-2], path[-1])) # direction from second last entry to last entry is start direction
        # goal parameters stay the same

        return new_path_problem

    def get_current_layout_solution(self, captured_state_grid: np.ndarray, parts_used: list, path: list):
        new_layout_solution = Solution(path=path, algorithm=None, parts=parts_used, path_problem=self._initial_path_problem,
                                       problem_solved=False, score = None, solution_grid=captured_state_grid)
        return new_layout_solution

    def get_invalid_solutions(self, remove:bool=True) -> list:
        """Gets invalid solutions and returns them. Optionally also removes them. Should be called after modifying the
         path problem."""

        #todo: code function




