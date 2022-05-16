import pathfinding_example
from function_test import app
from pathfinding_example import path_problem
from process_planning.process_planner import ProcessPlanner

if __name__ == "__main__":
    process_planner = ProcessPlanner(initial_path_problem=path_problem)
    app = app.FunctionTestApp(path_problem=pathfinding_example.path_problem, process_planner=process_planner)
