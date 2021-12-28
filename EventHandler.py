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
from path_finding.p_math import get_adjacency, sum_pos
from path_finding.common_types import Pos

from constants import directions

# todo: finish later
# todo: Questions: What information can we get? Only the current state of the grid? If a part has been placed?

standard_weights = Weights(1,1,1)
standard_algorithm = "mcsa*"


def is_outgoing_node(pos, pos_list):
    count = 0
    neighbor = get_neighbor(pos, pos_list)
    if neighbor:
            count += 1
    if count < 2:
        return True

def get_neighbor(pos, pos_list) -> Optional[Pos]:
    neighbor_positions = directions
    for neigh_pos in neighbor_positions:
        neighbor = sum_pos(pos, neigh_pos)
        if neighbor in pos_list:
            return neighbor




def get_trails_from_state_grid(state_grid: np.ndarray):
    # todo:
    #   1. np.ndenumerate state_grid, check for states, add nodes to respective set according to state
    #   2. go trough set where node_state==2, if one node is adjacent, they are part of a layout
    #   -> follow adjacent node until node is in 0/1 set -> interpret length
    #   (dont forget to remove node after expansion)

    trail_pos_list = []
    all_trails = []

    # sort nodes
    for pos, state in np.ndenumerate(state_grid):
        if state == 2:
            trail_pos_list.append(pos)

    neighbor_pos = directions

    trail_list = []
    for trail_pos in trail_pos_list:
        if is_outgoing_node(trail_pos, trail_pos_list):
            # new trail!
            trail = []
            trail_pos_list.remove(trail_pos)
            trail.append(trail_pos)
            next_pos = trail_pos

            while get_neighbor(next_pos, trail_pos_list):
                neighbor = get_neighbor(next_pos, trail_pos_list)
                next_pos = neighbor
                trail_pos_list.remove(neighbor)
                trail.append(neighbor)




            trail_list.append(trail)


        current_trail = []





    # # get first pos in list
    # while trail_pos_list:
    #     #get a start point (has only 1 neighbor)
    #     for pos in trail_pos_list:
    #         for neigh_pos in neighbor_pos:
    #
    #     start_pos = trail_pos_list[0]
    #     current_trail = [start_pos]
    #     while start_pos:
    #         for neigh_pos in neighbor_pos:
    #             neighbor = sum_pos(start_pos, neigh_pos)
    #
    #             if neighbor in trail_pos_list:
    #                 current_trail.append(neighbor)
    #                 trail_pos_list.remove(neighbor)
    #                 start_pos = neighbor
    #             else: start_pos = None
    #
    #     else:
    #         all_trails.append(current_trail)
    #         break
    #
    return all_trails






def grid_changed(captured_state_grid:np.ndarray , state_grid:np.ndarray):
    """Compares captured state_grid to another."""
    comparison = captured_state_grid == state_grid
    if not comparison.all():
        return True

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



class EventHandler:
    """Acts as an interface for handling events. Keeps track of the building process and provides new solutions on events."""
    def __init__(self, initial_path_problem: PathProblem, current_layout_solution: Optional[Solution], optimization_weights: Weights = standard_weights,
                 algorithm : str = standard_algorithm):

        self.completed_solutions_stack = {} # PathProblem: Solution

        self._initial_path_problem = initial_path_problem # original path problem
        self.optimal_solution = find_path(standard_weights, algorithm, self._initial_path_problem)

        self.latest_path_problem = deepcopy(initial_path_problem)

        self.current_layout_solution = current_layout_solution

        self.weights = optimization_weights
        self.algorithm = algorithm

    def grid_check(self, captured_state_grid: np.ndarray, parts_used : list, path: list) -> Optional[Solution]:
        """Checks for changes in the captured grid and returns a solution on change (if solvable)."""
        if grid_changed(captured_state_grid, self.latest_path_problem.state_grid):
            self.latest_path_problem = self.get_new_path_problem(state_grid=captured_state_grid, parts_used=parts_used, path=path)
            # todo: check if problem is solved/layout completed (last entry in path = goal)
            #  -> no new solution needed
            self.current_layout_solution = self.get_current_layout_solution(captured_state_grid,parts_used, path)

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




