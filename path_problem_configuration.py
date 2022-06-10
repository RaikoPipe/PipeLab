from path_finding.grid import grid_functions, randomizer
from path_finding.pf_data_class.path_problem import PathProblem
from path_finding.pf_data_class.weights import Weights
from path_finding.search_algorithm import find_path
from type_dictionary.constants import fitting_id

# Configuration

x = 10 #: Horizontal shape (amount of nodes)
y = 25 #: Vertical shape (amount of nodes)

transition_points_set = {(-2, 16)} #: Set containing transition points that show where a transition from one wall to another occurs.

start_pos = (3, 0) #: Start position of the search.
goal_pos = (5, 24) #: Goal position that needs to be reached

start_directions = {(0, 1), (1, 0), (-1, 0), (0, -1)} #: Set containing directions that restrict in which direction the search algorithm can start the search.
goal_directions = {(0, 1), (1, 0), (-1, 0), (0, -1)} #: Set containing directions that restrict in which direction the search algorithm can append to the goal.

pipe_stock = {0: 10, 2: 10, 3: 10} #: Part IDs pointing the amount of parts available for assembly.

part_cost = {0: 5.32, 1: 1.15, 2: 1.38, 3: 1.60, 4: 1.82, 5: 2.04, 6: 2.50} #: Part IDs pointing to their cost value.

state_grid = grid_functions.get_empty_stategrid(x, y) #: A grid array where each position in the grid points to an integer value representing a state.

algorithm="mcsa*" #: Defines how scores are calculated and therefore how nodes are evaluated in the search.
weights = Weights(path_length=1, cost=0, distance_to_obstacles=0) #: Weights used if search algorithm is a multi-criteria search algorithm (mca*, mcsa*)

new_path_problem = PathProblem(state_grid=state_grid, start_pos=start_pos, goal_pos=goal_pos,
                               start_directions=start_directions,
                               goal_directions=goal_directions, part_cost=part_cost,
                               starting_part=fitting_id, part_stock=pipe_stock, weights=weights, algorithm=algorithm,
                               transition_points=transition_points_set)


def get_solvable_path_problem_with_random_obstacles(obs_frequency) -> PathProblem:
    """Tries to find a solvable random obstacle configuration for the path problem defined in this file.
    Raises an exception after 1000 unsuccessful tries.

    Args:
        obs_frequency(:obj:`float`): Chance for a node to be an obstacle.

    Returns:
        :class:`PathProblem<path_problem>`
        """
    solution = None
    count = 0

    while solution is None:

        if count == 1000:
            raise Exception("There probably is no single solvable problem configuration for the given parameters!")

        state_grid = grid_functions.get_empty_stategrid(x, y)

        state_grid = randomizer.set_random_obstacles(obs_frequency,
                                                     state_grid,
                                                     transition_points_set)
        # clear start and goal of potential obstacles
        state_grid[start_pos] = 0
        state_grid[goal_pos] = 0
        new_path_problem.state_grid = grid_functions.set_transition_points(state_grid, transition_points_set)

        solution = find_path(path_problem=new_path_problem, draw_path=False)

        if solution:
            return new_path_problem
        else:
            print("No solution found! Trying again...")


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
