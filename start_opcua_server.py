import asyncio
import logging

import pathfinding_example
from opc_ua.server import main

from process_planning.process_planner import ProcessPlanner

logging.basicConfig(level=logging.INFO)


def start_server(process_planner):
    asyncio.run(main(process_planner))


if __name__ == "__main__":
    start_server(ProcessPlanner(initial_path_problem=pathfinding_example.get_solvable_path_problem_with_random_obstacles()))
