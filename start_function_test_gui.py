import pathfinding_example
from FunctionTestGUI import GUI
from pathfinding_example import path_problem
from ProcessPlanning.ProcessPlanner import ProcessPlanner

if __name__ == "__main__":
    process_planner = ProcessPlanner(initial_path_problem=path_problem)
    app = GUI.FunctionTestGUI(path_problem=pathfinding_example.path_problem, initial_state=None, process_planner=process_planner)
