

import asyncio
import logging

from OPC_UA.Server import main
from pathfinding_example import path_problem
from ProcessPlanning.ProcessPlanner import ProcessPlanner

logging.basicConfig(level=logging.INFO)

if __name__ == "__main__":
    process_planner = ProcessPlanner(initial_path_problem=path_problem)
    asyncio.run(main(process_planner))
