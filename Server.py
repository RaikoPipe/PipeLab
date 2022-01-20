"""Server; Instantiates Interpreter, Process Planner"""
from ProcessPlanner import ProcessPlanner


"""Debug starts here"""
from grid import grid_functions, randomizer
from data_class.Weights import Weights
from data_class.PathProblem import PathProblem

x=20
y=20

r_grid, mounting_wall_data = grid_functions.get_rendering_grid(x,y)
solution = None
while solution is None:
    state_grid = grid_functions.get_empty_stategrid(x, y)
    state_grid = randomizer.set_random_obstacles(0.1, state_grid)
    start_node = (0,0)
    goal_node = (17,19)
    state_grid[start_node] = 0
    state_grid[goal_node] = 0

    pipe_stock = {0: 100, 1: 100, 2: 100, 3: 100, 4: 100, 5: 100, 6: 100}

    part_cost = {0: 5.32, 1: 3.00, 2: 3.00, 3: 3.00, 4: 3.00, 5: 3.00, 6: 00}

    weights = Weights(path_length=1, cost=0, distance_to_obstacles=0)

    path_problem = PathProblem(state_grid=state_grid, start_node=start_node, goal_node=goal_node, start_direction=(1, 0),
                               goal_direction=(0, 1), goal_is_transition=False, part_cost=part_cost,
                               starting_part=None, part_stock=pipe_stock, weights=weights, algorithm="mcsa*")
"""Debug ends here"""

class someServerEnv:
    """Server Class. Waits for new input, gets instructions from process planner"""

    def __init__(self):
        self.current_instruction = ""
        self.process_planer = ProcessPlanner(initial_path_problem=path_problem, initial_state=None)

    def get_instructions(self):
        #todo: give process_planner new data, receive instructions, give instructions to clients
        process_planer.event_captured_state_grid()
