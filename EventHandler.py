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
from path_finding.common_types import *

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




def get_trails_from_state_grid(state_grid: np.ndarray) -> list[Trail]:
    trail_pos_list = []

    # sort nodes
    for pos, state in np.ndenumerate(state_grid):
        if state == 2:
            trail_pos_list.append(pos)

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

    return trail_list

#todo: interpret trail as path, parts_used

# def get_path_from_trail(trail:Trail, part_stock: dict[int:int]):
#     # todo: go through trail; get length of one direction; at first, we assume there are no straight pipes with len=1
#     path = []
#     current_direction = None
#     length = 0
#     for idx, pos in enumerate(trail):
#         previous_direction = current_direction
#         a = pos
#         b = trail[idx+1]
#         current_direction = diff_pos(a,b) # diff of two adjacent trail pos is direction
#
#         if current_direction == previous_direction or previous_direction is None:
#             length += 1
#         else:
#             path.append()

def get_path_from_trail(trail: Trail):
    # todo: go through trail; get length of one direction; at first, we assume there are no straight pipes with len=1
    path = []

    while trail:
        pos = trail[0]
        path.append(pos)

        next_pos = trail[1]

        direction = diff_pos(pos, next_pos)
        while sum_pos(pos, direction) in trail:
            trail.pop(0)
            pos = trail[0]
        else:
            path.append(pos)
            trail.pop(0)

    return path

def correct_path_start(path:Path, part_stock:dict[int:int]):
    """Corrects the beginning of path if the part isn't available in the stock by adding a corner."""
    a = path[0]
    b = path[1]
    length = abs(b[0]-a[0])+abs(b[1]-a[1])
    part = part_stock.get(length)
    if part is None or part == 0:
        direction = get_direction(diff_pos(a,b))
        path.insert(0, a)
        path[1] = sum_pos(a,direction)
        part_stock[length-1] -= 1
        part_stock[0] -= 1
    return path, part_stock

def event_part_removed(part_id,part_stock):
    part_stock[part_id] -= 1
    return part_stock

def event_part_placed():
    # todo: check where part has been placed and if it is adjacent to a current layout and valid -> expand this path
    # todo: check if paths have been connected: fuse into one path
    # todo: check if current_state_grid overlaps with optimal_solution_state_grid

def event_part_removed():
    #todo: check where part was removed and if path was split or just reduced

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




