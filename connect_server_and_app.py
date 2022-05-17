import pathfinding_example
import start_opcua_server
import start_function_test_app
from process_planning.process_planner import ProcessPlanner
import threading


if __name__ == "__main__":
    process_planner = ProcessPlanner(initial_path_problem=
                                     pathfinding_example.get_solvable_path_problem_with_random_obstacles())
    x = threading.Thread(target=start_opcua_server.start_server, args=(process_planner,), daemon=True)
    x.start()
    start_function_test_app.start_app(process_planner, update_self_periodically=True)
