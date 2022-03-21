import numpy as np
from typing import Optional
from constants import valid_directions
from path_finding.common_types import *
from path_finding.path_math import get_adjacency, sum_pos, diff_pos, get_direction

"""Gets data (state_grids, part stock...), interprets it and returns events"""

"""Construction Event Codes
    -2: Error: Unknown Error
    -1: Error: more than 1 construction change occurred in state grid
    0: No construction event occurred.
    1: A part was removed from the mounting wall
    2: A part previously picked was placed"""







def check_action_event(picked_parts:list, placed_parts:list, latest_state_grid, captured_state_grid):

    #todo: get difference between both state_grids
    # more than 1 action occurred: Calculate path again
    diff = captured_state_grid - latest_state_grid
    added_trails = get_trails_from_state_grid(diff, 2)
    removed_trails = get_trails_from_state_grid(diff, -2)

    event_code = -2
    part_id = None
    trail = None


    if added_trails and removed_trails or len(added_trails) > 1 or len(removed_trails) > 1:
        event_code = -1

    elif added_trails:
        #one part has been added
        trail = added_trails[0]
        part_id = len(trail)
        if part_id in picked_parts:
            event_code = 2
        else:
            event_code = -1

    elif removed_trails:
        trail = removed_trails[0]
        part_id = len(trail)
        if part_id in placed_parts:
            event_code = 1
        else:
            event_code = -1

    return {"code": event_code, "part_id" :None, "trail" : trail}

def grid_changed(captured_state_grid: np.ndarray, latest_state_grid: np.ndarray) -> bool:
    """Returns true if captured_state_grid has changed"""
    comparison = captured_state_grid == latest_state_grid
    if not comparison.all():
        return True

def is_outgoing_node(pos, pos_list) -> bool:
    """Checks if pos is an outgoing node"""
    count = 0
    neighbor = get_neighbor(pos, pos_list)
    if neighbor:
            count += 1
    if count < 2:
        return True

def get_neighbor(pos, pos_list) -> Optional[Pos]:
    """Returns neighbor of pos if in pos_list else None."""
    neighbor_positions = valid_directions
    for neigh_pos in neighbor_positions:
        neighbor = sum_pos(pos, neigh_pos)
        if neighbor in pos_list:
            return neighbor

def get_trails_from_state_grid(state_grid: np.ndarray, searched_state: int) -> list[Trail]:
    trail_pos_list = []

    # sort nodes
    for pos, searched_state in np.ndenumerate(state_grid):
        if searched_state == searched_state:
            trail_pos_list.append(pos)

    trail_list = []
    for trail_pos in trail_pos_list:
        if is_outgoing_node(trail_pos, trail_pos_list):
            # new trail!
            trail = []
            trail_pos_list.remove(trail_pos)
            trail.append(trail_pos)
            next_pos = trail_pos

            while get_neighbor(next_pos, trail_pos_list):
                neighbor = get_neighbor(next_pos, trail_pos_list)
                next_pos = neighbor
                trail_pos_list.remove(neighbor)
                trail.append(neighbor)




            trail_list.append(trail)

    return trail_list

#todo: interpret trail as path, parts_used

# def get_path_from_trail(trail:Trail, part_stock: dict[int:int]):
#     # todo: go through trail; get length of one direction; at first, we assume there are no straight pipes with len=1
#     path = []
#     current_direction = None
#     length = 0
#     for idx, pos in enumerate(trail):
#         previous_direction = current_direction
#         a = pos
#         b = trail[idx+1]
#         current_direction = diff_pos(a,b) # diff of two adjacent trail pos is direction
#
#         if current_direction == previous_direction or previous_direction is None:
#             length += 1
#         else:
#             path.append()

def get_path_from_trail(trail: Trail):
    # todo: go through trail; get length of one direction; at first, we assume there are no straight pipes with len=1
    path = []

    while trail:
        pos = trail[0]
        path.append(pos)

        next_pos = trail[1]

        direction = diff_pos(pos, next_pos)
        while sum_pos(pos, direction) in trail:
            trail.pop(0)
            pos = trail[0]
        else:
            path.append(pos)
            trail.pop(0)

    return path

def correct_path_start(path:Path, part_stock:dict[int:int]):
    """Corrects the beginning of path if the part isn't available in the stock by adding a corner."""
    a = path[0]
    b = path[1]
    length = abs(b[0]-a[0])+abs(b[1]-a[1])
    part = part_stock.get(length)
    corrected = False
    if part is None or part == 0:
        direction = get_direction(diff_pos(a,b))
        path.insert(0, a)
        path[1] = sum_pos(a,direction)
        part_stock[length-1] -= 1
        part_stock[0] -= 1
        corrected = True
    return path, part_stock, corrected

def get_definite_path_from_path(path:Path, part_stock:dict[int:int]) -> DefinitePath:
    """Returns a DefinitePath from a list of paths. (End of a path is identified by None)"""
    definite_path = []
    path, _, corrected = correct_path_start(path, part_stock=part_stock)
    a = path[0]
    if corrected:
        definite_path.append((a, 0))
        start = 1
    else:
        start = 0

    for idx, a in enumerate(path, start=start):
        if idx == len(path):
            definite_path.append((a, None))
            break
        b = path[idx+1]
        part_id = abs(b[0]-a[0])+abs(b[1]-a[1])
        definite_path.append((a, part_id))

    return definite_path



def event_part_removed(part_id,part_stock):
    part_stock[part_id] -= 1
    return part_stock