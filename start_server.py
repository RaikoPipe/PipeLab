

import asyncio
import logging

from opc_ua.server import main
from pathfinding_example import path_problem
from process_planning.process_planner import ProcessPlanner

logging.basicConfig(level=logging.INFO)

if __name__ == "__main__":
    process_planner = ProcessPlanner(initial_path_problem=path_problem)
    asyncio.run(main(process_planner))
