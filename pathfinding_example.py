from path_finding.grid import grid_functions, randomizer
from path_finding.pf_data_class.path_problem import PathProblem
from path_finding.pf_data_class.weights import Weights
from path_finding.search_algorithm import find_path
from type_dictionary.constants import fitting_id

# Configuration

x = 10
y = 18

transition_points_set = {(-2, 10)}

start_node = (0, 0)
goal_node = (9, 17)
start_directions = {(0, 1), (1, 0), (-1, 0), (0, -1)}
goal_directions = {(0, 1), (1, 0), (-1, 0), (0, -1)}

pipe_stock = {0: 10, 1: 10, 2: 10, 3: 10, 4: 10, 5: 10, 6: 10}

part_cost = {0: 5.32, 1: 1.15, 2: 1.38, 3: 1.60, 4: 1.82, 5: 2.04, 6: 2.50}

state_grid = None

algorithm="mcsa*"
weights = Weights(path_length=0, cost=1, distance_to_obstacles=0)

obs_frequency = 0.1

tentative_path_problem = PathProblem(state_grid=state_grid, start_pos=start_node, goal_pos=goal_node,
                                     start_directions=start_directions,
                                     goal_directions=goal_directions, part_cost=part_cost,
                                     starting_part=fitting_id , part_stock=pipe_stock, weights=weights, algorithm=algorithm,
                                     transition_points=transition_points_set)


def get_solvable_path_problem_with_random_obstacles():
    solution = None
    count = 0
    while solution is None:

        state_grid = grid_functions.get_empty_stategrid(x, y)
        state_grid[start_node] = 0
        state_grid[goal_node] = 0
        state_grid = randomizer.set_random_obstacles(obs_frequency,
                                                     state_grid,
                                                     transition_points_set)
        tentative_path_problem.state_grid = grid_functions.set_transition_points(state_grid, transition_points_set)

        solution = find_path(path_problem=tentative_path_problem)

        if solution:
            return tentative_path_problem

        if count == 1000:
            raise Exception("There probably is no single solvable problem configuration for the given parameters!")

        count += 1

# Vpython rendering
# print(path_problem)
# print(solution)

# vpython_rendering

# scene = object_rendering.create_new_scene()
# mounting_wall, dot_object = \
#     object_rendering.render_mounting_wall(scene=scene, rendering_grid=r_grid, mounting_wall_data=mounting_wall_data)
#
# obstacles = group_rendering.render_obstacles_from_state_grid(state_grid=state_grid, rendering_grid=r_grid, scene=scene)
# start_marker, goal_marker = group_rendering.render_start_goal_markers(scene=scene,start_pos=start_node, goal_pos=goal_node,rendering_grid=r_grid)
#
# solution_layout = group_rendering.render_pipe_layout(solution.definite_path, r_grid, scene, opacity=.5)
#
# print(solution.definite_path[-2])

# positions = debug_rendering.display_pos(rendering_grid=r_grid, scene=scene)
