import asyncio
import time
from ProcessPlanner import ProcessPlanner
from grid.grid_functions import get_empty_stategrid, change_grid_states
from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
import traceback

s = get_empty_stategrid(20,20)
part_stock = {0:20,1:20,2:20,3:20,4:20}
part_cost = {0:5,1:2,2:2.5,3:2.5,4:3}
initial_path_problem = PathProblem(state_grid=s, start_node=(1,1), goal_node=(9,9), start_direction=(0,1), goal_direction=(0,-1),
                                   #upwards (0,1), downwards(0,-1), right(1,0), left(-1,0)
                                   starting_part=None, goal_is_transition=False, part_stock=part_stock,part_cost=part_cost)
event_handler = ProcessPlanner(initial_path_problem=initial_path_problem, initial_state=None)

try:
    change_grid_states(s, [((1,1),2),((1,2),2),((1,3),2),((1,4),2)])
except:
    traceback.print_exc()

async def event_scheduler():
    """coroutine to schedule a random capture event"""
    path=[]
    part_used = []
    for index, node in enumerate(Solution.path):
        await asyncio.sleep(1)
        #captured input
        path.append(node)
        part_used.append(Solution.parts[index])
        #todo: change state_grid according to path
        state_grid = change_grid_states(s, [((1,1),2),((1,2),2),((1,3),2),((1,4),2)])
        # check if anything has changed
        new_solution = event_handler.grid_check(path=path, captured_state_grid=state_grid, parts_used=part_used)

        return new_solution


print(asyncio.run(event_scheduler()))




