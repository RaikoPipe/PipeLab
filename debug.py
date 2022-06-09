from path_finding.grid import grid_functions, randomizer
from path_finding.pf_data_class.path_problem import PathProblem
from path_finding.pf_data_class.weights import Weights
from path_finding.search_algorithm import find_path
from process_planning.pp_data_class.process_output import ProcessOutput
from process_planning.process_planner import ProcessPlanner
from type_dictionary import constants
from type_dictionary.constants import fitting_id

"""Debug File to catch evil bugs."""


# Configuration

x = 10 #: Horizontal shape (amount of nodes)
y = 25 #: Vertical shape (amount of nodes)

transition_points_set = {(-2, 16)} #: Set containing transition points that show where a transition from one wall to another occurs.

start_pos = (3, 0) #: Start position of the search.
goal_pos = (5, 24) #: Goal position that needs to be reached

start_directions = {(0, 1), (1, 0), (-1, 0), (0, -1)} #: Set containing directions that restrict in which direction the search algorithm can start the search.
goal_directions = {(0, 1), (1, 0), (-1, 0), (0, -1)} #: Set containing directions that restrict in which direction the search algorithm can append to the goal.

pipe_stock = {0: 10, 2: 10, 4: 10} #: Part IDs pointing the amount of parts available for assembly.

part_cost = {0: 5.32, 1: 1.15, 2: 1.38, 3: 1.60, 4: 1.82, 5: 2.04, 6: 2.50} #: Part IDs pointing to their cost value.

state_grid = grid_functions.get_empty_stategrid(x, y) #: A grid array where each position in the grid points to an integer value representing a state.

algorithm="mcsa*" #: Defines how scores are calculated and therefore how nodes are evaluated in the search.
weights = Weights(path_length=1, cost=1, distance_to_obstacles=0) #: Weights used if search algorithm is a multi-criteria search algorithm (mca*, mcsa*)

new_path_problem = PathProblem(state_grid=state_grid, start_pos=start_pos, goal_pos=goal_pos,
                               start_directions=start_directions,
                               goal_directions=goal_directions, part_cost=part_cost,
                               starting_part=fitting_id, part_stock=pipe_stock, weights=weights, algorithm=algorithm,
                               transition_points=transition_points_set)

solution = None
count = 0

obs_frequency = 0

while True:

    state_grid = grid_functions.get_empty_stategrid(x, y)
    state_grid[start_pos] = 0
    state_grid[goal_pos] = 0
    state_grid = randomizer.set_random_obstacles(obs_frequency,
                                                 state_grid,
                                                 transition_points_set)
    state_grid[start_pos] = 0
    state_grid[goal_pos] = 0
    new_path_problem.state_grid = grid_functions.set_transition_points(state_grid, transition_points_set)

    solution = find_path(path_problem=new_path_problem)

    if solution:
        print("Solution found!")
        path_length = len(solution.node_trail)
        cost: float = 0
        for _, part_id in solution.node_path:
            cost += part_cost[part_id]
        print("Len: " + str(path_length))
        print("Cost: " + str(cost))
        break

    else:
        print("No solution found!")

    count += 1

process_planner = ProcessPlanner(initial_path_problem=new_path_problem)

motion_events = [(0, None, 4), (0, None, 4), (0, None, 4), (0, None, 4), (2, None, 4), (4, None, 4), (3, 2, 3),
                 (3, 2, 2), (3, 0, 1), (3, 5, 1), (4, 5, 3), (4, 5, 2), (6, 5, 1)]
for item in motion_events:
    x = item[0]
    y = item[1]
    code = item[2]
    if code in {constants.pick_manual_event_code, constants.pick_robot_event_code}:
        output: ProcessOutput = process_planner.handle_motion_event((x, code))
    elif code in {constants.fit_event_code, constants.pipe_event_code, constants.att_event_code}:
        output = process_planner.handle_motion_event(((x, y), code))
