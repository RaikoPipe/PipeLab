import path_problem_configuration
import run_opcua_server
import run_function_test_app
from process_planning.process_planner import ProcessPlanner
import threading


if __name__ == "__main__":
    path_problem = path_problem_configuration.get_solvable_path_problem_with_random_obstacles(.1)

    process_planner = ProcessPlanner(initial_path_problem= path_problem)

    x = threading.Thread(target=run_opcua_server.run_server, args=(process_planner,), daemon=True)
    x.start()

    run_function_test_app.run_app(process_planner, update_self_periodically=True)
