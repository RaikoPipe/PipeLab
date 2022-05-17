import asyncio
import logging

from opc_ua.server import main
from pathfinding_example import path_problem
from process_planning.process_planner import ProcessPlanner

logging.basicConfig(level=logging.INFO)


def start_server(process_planner):
    asyncio.run(main(process_planner))


if __name__ == "__main__":
    start_server(ProcessPlanner(initial_path_problem=path_problem))
