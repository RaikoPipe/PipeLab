from path_problem_example import path_problem
from function_test import Server

import asyncio
import logging


logging.basicConfig(level=logging.INFO)
asyncio.run(Server.main(path_problem))