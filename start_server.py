from __future__ import annotations

import asyncio
import logging

from OPC_UA import Server
from pathfinding_example import path_problem

logging.basicConfig(level=logging.INFO)
asyncio.run(Server.main(path_problem))
