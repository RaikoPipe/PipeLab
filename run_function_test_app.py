import path_problem_configuration
from function_test import app
from process_planning.process_planner import ProcessPlanner


def run_app(process_planner, update_self_periodically=False):
    app.FunctionTestApp(path_problem=process_planner.last_process_state.aimed_solution.path_problem,
                        process_planner=process_planner, update_self_periodically=update_self_periodically)


if __name__ == "__main__":
    path_problem = path_problem_configuration.get_solvable_path_problem_with_random_obstacles(0.2)

    run_app(ProcessPlanner(initial_path_problem=path_problem))
