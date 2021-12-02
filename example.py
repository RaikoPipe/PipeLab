import asyncio
import time
import EventHandler
from grid.grid_functions import get_empty_stategrid, change_grid_states

s = get_empty_stategrid(10,10)
eventhandler = EventHandler()

async def do_after():
    await asyncio.sleep(1)
    path = [(4,0),(4,3)]
    state_grid = change_grid_states(s, [((4,0),2),((4,1),2),((4,2),2),((4,3),2)])
    part_used = [4]

    new_solution = eventhandler.grid_check(path=path, captured_state_grid=state_grid, parts_used=part_used)


asyncio.run(do_after())




