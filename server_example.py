from __future__ import annotations

from path_problem_example import path_problem
from OPC_UA import Server

import asyncio
import logging

logging.basicConfig(level=logging.INFO)
asyncio.run(Server.main(path_problem))
