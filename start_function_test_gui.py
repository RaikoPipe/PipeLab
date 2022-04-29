import path_problem_example
from function_test import GUI

app = GUI.function_test_app(state_grid=path_problem_example.solution.state_grid, path_problem=path_problem_example.path_problem, initial_state=None, use_dark_theme=False)