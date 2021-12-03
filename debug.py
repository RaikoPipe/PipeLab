from path_finding.search_algorithm import find_path
from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from grid.grid_functions import get_empty_stategrid, change_grid_states

state_grid = get_empty_stategrid(20,20)
#todo: use change_grid_states function to create a mounting wall with obstacles
#todo: create a solution with a* and extract path and parts to define a static solution

pipe_stock= {0:100,1:100, 2:100, 3:100, 4:100, 5:100, 6:100}

part_cost = {0: 5.32, 1: 3.00, 2:3.00, 3:3.00, 4:3.00, 5:3.00, 6:3.00}

path_problem = PathProblem(state_grid = state_grid, start_node=(0,0), goal_node=(19,9), start_direction=(1,0),
                           goal_direction=(-1,0), goal_is_transition=False, part_cost=part_cost,
                           starting_part=None, part_stock=pipe_stock)

weights = Weights(path_length=1, cost=0, distance_to_obstacles=0)

solution = find_path(weights, "mca*", path_problem=path_problem)

print(solution)