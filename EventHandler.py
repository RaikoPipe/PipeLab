from vpython import vector, color
import numpy as np
import rendering
from path_finding.search_algorithm import find_path
from grid import grid_functions
from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
from data_class.Weights import Weights
from copy import copy
from path_finding.restriction_functions import get_direction_of_pos, get_worst_move_cost
from path_finding.p_math import diff_nodes
from path_finding.search_algorithm import find_path
from typing import Optional

# todo: finish later
# todo: Questions: What information can we get? Only the current state of the grid? If a part has been placed?

standard_weights = Weights(1,1,1)
standard_algorithm = "mcsa*"

def grid_changed(captured_state_grid, state_grid):
    """Compares captured state_grid to another."""
    if captured_state_grid != state_grid:
        return True

def get_current_part_stock(part_stock, parts_used):
    """Returns the part stock sans the parts already used."""
    for part_id in parts_used:
        part_stock[part_id] -= 1

    return part_stock

def get_new_solution(path_problem, weights):
    return find_path(path_problem=path_problem, weights=weights, algorithm=standard_algorithm)

#todo: it is assumed that the sensor detects changes on the grid and the layout (current path and currently used parts)


def check_solution_stack(completed_solutions_stack, current_path_problem) -> Optional[Solution]:
    for path_problem in completed_solutions_stack:
        if path_problem == current_path_problem:
            return completed_solutions_stack[path_problem]
        else: return None



class EventHandler:
    solution_list: dict  # {solution: score}
    """Acts as an interface for handling events. Keeps track of the building process and provides new solutions on events."""
    def __init__(self, initial_path_problem: PathProblem, optimization_weights: Weights = standard_weights,
                 algorithm : str = standard_algorithm):

        self.completed_solutions_stack = {} # PathProblem: Solution

        self._path_problem = initial_path_problem # original path problem
        self.optimal_solution = find_path(standard_weights, algorithm, self._path_problem)

        self.latest_path_problem = copy(initial_path_problem)
        self.previous_path_problem : PathProblem

        self.latest_layout: Solution

        self.weights = optimization_weights
        self.algorithm = algorithm

    def grid_check(self, captured_state_grid, parts_used, path) -> Optional[Solution]:
        if grid_changed(captured_state_grid, self.latest_path_problem.state_grid):
            self.latest_path_problem = self.get_new_path_problem(state_grid=captured_state_grid, parts_used=parts_used, path=path)

            solution = check_solution_stack(self.completed_solutions_stack, self.latest_path_problem)
            if solution is None:
                solution = get_new_solution(path_problem=self.latest_path_problem, weights=self.weights)
            return solution
        return None # grid has not changed, nothing to do.

    def get_new_path_problem(self, state_grid, parts_used:dict, path:list) -> PathProblem:
        """Returns a new path problem according to detected changes"""

        new_path_problem = copy(self._path_problem) # make a copy of the original path problem

        current_pipe_stock = get_current_part_stock(copy(self._path_problem.part_stock), parts_used)
        new_path_problem.part_stock = current_pipe_stock
        new_path_problem.state_grid = state_grid
        new_path_problem.start_node = path[-1] # last entry is new start pos
        new_path_problem.start_direction = get_direction_of_pos(diff_nodes(path[-2], path[-1])) # direction from second last entry to last entry is start direction
        # goal parameters stay the same

        return new_path_problem



    def get_invalid_solutions(self, remove:bool=True) -> list:
        """Gets invalid solutions and returns them. Optionally also removes them. Should be called after modifying the
         path problem."""

        #todo: code function




