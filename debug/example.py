from data_class.PathProblem import PathProblem
from data_class.Weights import Weights
from function_test_gui import function_test_app
from grid import grid_functions, randomizer
from path_finding.search_algorithm import find_path

# todo: use change_grid_states function to create a mounting wall with obstacles
# todo: create a solution with a* and extract path and parts to define a static solution

x = 10
y = 20

r_grid, mounting_wall_data = grid_functions.get_rendering_grid(x, y)
solution = None
while solution is None:

    state_grid = grid_functions.get_empty_stategrid(x, y)
    state_grid = randomizer.set_random_obstacles(0., state_grid)
    start_node = (0, 0)
    goal_node = (9, 17)
    state_grid[start_node] = 0
    state_grid[goal_node] = 0

    pipe_stock = {0: 100, 1: 100, 2: 100, 3: 100, 4: 100, 5: 100, 6: 100}

    part_cost = {0: 5.32, 1: 3.00, 2: 3.00, 3: 3.00, 4: 3.00, 5: 3.00, 6: 3.00}

    weights = Weights(path_length=1, cost=1, distance_to_obstacles=0)

    path_problem = PathProblem(state_grid=state_grid, start_pos=start_node, goal_pos=goal_node,
                               start_directions={(0, 1), (1, 0), (-1, 0), (0, -1)},
                               goal_directions={(0, 1), (1, 0), (-1, 0), (0, -1)}, part_cost=part_cost,
                               starting_part=0, part_stock=pipe_stock, weights=weights, algorithm="mcsa*")

    solution = find_path(path_problem=path_problem)

    if solution is None:
        print("No solution found")
    else:
        "Solution found!"

# print(path_problem)
# print(solution)

# rendering

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

app = function_test_app(state_grid=solution.state_grid, path_problem=path_problem, initial_state=None)
