from path_finding.find_path import PathFinder
from path_finding.path_data_classes import PathProblem, Weights
from grid.grid_functions import get_empty_stategrid, change_grid_states

state_grid = get_empty_stategrid(20,20)

pipe_stock= {0:1000,1:1000}

path_problem = PathProblem(state_grid = state_grid, start_pos=(0,0), goal_pos=(20,20), start_axis=(1,0),
                           goal_axis=(-1,0), goal_is_transition=False, pipe_stock=pipe_stock)

weights = Weights(path_length=1, cost=1, distance_to_obstacles=1)

path_finder = PathFinder(path_problem)

path_finder.find_path(weights, "mca*")