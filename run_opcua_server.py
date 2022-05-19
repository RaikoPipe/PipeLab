import asyncio
import logging

import path_problem_configuration
from opc_ua.server import main

from process_planning.process_planner import ProcessPlanner

logging.basicConfig(level=logging.INFO)


def run_server(process_planner):
    asyncio.run(main(process_planner))


if __name__ == "__main__":
    path_problem = path_problem_configuration.get_solvable_path_problem_with_random_obstacles(.1)

    run_server(ProcessPlanner(initial_path_problem=path_problem))
