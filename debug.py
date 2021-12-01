from path_finding.search_algorithm import PathFinder
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from grid.grid_functions import get_empty_stategrid, change_grid_states

state_grid = get_empty_stategrid(20,20)

pipe_stock= {0:1000,1:1000, 2:1000, 3:1000, 4:1000, 5:1000, 6:1000}

part_cost = {0: 5.32, 1: 3.00, 2:3.00, 3:3.00, 4:3.00, 5:3.00, 6:3.00}

path_problem = PathProblem(state_grid = state_grid, start_node=(0,0), goal_node=(8,9), start_direction=(1,0),
                           goal_direction=(-1,0), goal_is_transition=False, pipe_stock=pipe_stock, part_cost=part_cost,
                           starting_part=None)

weights = Weights(path_length=1, cost=0, distance_to_obstacles=0)

path_finder = PathFinder(path_problem)

solution = path_finder.find_path(weights, "mca*")

print(solution)