import pathfinding_example
from function_test import app
from pathfinding_example import path_problem
from process_planning.process_planner import ProcessPlanner


def start_app(process_planner, update_self_periodically=False):
    app.FunctionTestApp(path_problem=pathfinding_example.path_problem, process_planner=process_planner, update_self_periodically=update_self_periodically)


if __name__ == "__main__":
    start_app(ProcessPlanner(initial_path_problem=path_problem))
