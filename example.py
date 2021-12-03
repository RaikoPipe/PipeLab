import asyncio
import time
from EventHandler import EventHandler
from grid.grid_functions import get_empty_stategrid, change_grid_states
from data_class.PathProblem import PathProblem

s = get_empty_stategrid(20,20)
part_stock = {0:20,1:20,2:20,3:20,4:20}
part_cost = {0:5,1:2,2:2.5,3:2.5,4:3}
initial_path_problem = PathProblem(state_grid=s, start_node=(1,1), goal_node=(9,9), start_direction=(0,1), goal_direction=(0,-1),
                                   starting_part=None, goal_is_transition=False, part_stock=part_stock,part_cost=part_cost)
event_handler = EventHandler(initial_path_problem=initial_path_problem, current_layout_solution=None)

async def event_scheduler():
    """coroutine to schedule a random capture event"""
    await asyncio.sleep(1)
    path = [(1,1),(1,5)]
    state_grid = change_grid_states(s, [((1,1),2),((1,2),2),((1,3),2),((1,4),2)])
    part_used = [4]

    new_solution = event_handler.grid_check(path=path, captured_state_grid=state_grid, parts_used=part_used)

    return new_solution


print(asyncio.run(event_scheduler()))




