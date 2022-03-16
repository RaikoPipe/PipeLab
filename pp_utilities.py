import event_interpreting
from data_class.LayoutState import LayoutState
from data_class.Solution import Solution
from data_class.State import State
from path_finding.common_types import Pos, Trail, DefinitePath


def picking_robot_has_wrong_part(layout_state: LayoutState, carried_part_id):
    """Check if the currently carried part can be used to assist in the current layout"""
    if layout_state.pipe_id == carried_part_id and not layout_state.pipe_set:

def determine_next_part(layout_state:LayoutState):
    next_part_id = None

    if not layout_state.att_set:
        # offer attachment
        next_part_id = -1
    elif not layout_state.pipe_set:
        next_part_id = layout_state.pipe_id
    elif len(layout_state.fit_set) < 2:
        next_part_id = 0

    return next_part_id


def get_current_path(state_grid, part_stock):
    """Returns the current paths from a state_grid"""
    trail_list = event_interpreting.get_trails_from_state_grid(state_grid=state_grid, searched_state=2)
    path_list = []
    for trail in trail_list:
        path = event_interpreting.get_path_from_trail(trail)
        path, _ = event_interpreting.correct_path_start(path, part_stock)
        path_list.append(path)
    return path_list


def get_updated_motion_dict(new_pos, motion_dict):
    for pos, event in new_pos.items():
        if pos == 0 and pos in motion_dict:
            motion_dict.pop(pos)
        else:
            motion_dict[pos] = event

    return motion_dict


def deviated_from_path(current_state: State, optimal_solution: Solution):

    for connection in current_state.fc_set:
        if connection not in optimal_solution.fc_set:
            return True
    else:
        return False


def get_deviation_trail(length:int, direction:Pos, fit_pos:tuple) -> Trail:
    # create new state

    # if any of the following pos is None, there was an error
    pipe_start_pos = None
    pipe_end_pos = None

    # update state grid
    construction_trail = []

    #set construction_parts fittings
    construction_parts = {fit_pos[0]: 0, fit_pos[1]: 0}

    for i in range(length + 1):
        state_pos = (fit_pos[0][0] + direction[0] * i, fit_pos[0][1] + direction[1] * i)
        construction_trail.append(state_pos)

    return construction_trail


def get_deviation_state(length:int, att_set:set, pipe_set:set, fit_tup:tuple):
    pipe_id = length - 2
    rec_att_pos = (0,0) #fixme: get recommended att_pos

    layout_state = LayoutState(att_set=att_set, pipe_set=pipe_set, fit_set={fit_tup[0], fit_tup[1]},
                               pipe_id=pipe_id, correct_fitting_pos=(fit_tup[0], fit_tup[1]), recommended_attachment_pos=rec_att_pos,
                               completed = True)

    return layout_state


def get_updated_state_on_construction_event(tentative_state : State, fit_diff, fit_dir, new_pos, pos) -> State:
    # create new state

    # update fitting_connections
    tentative_state.connection_count[new_pos] = 1
    tentative_state.connection_count[pos] += 1

    # update fitting_pos
    tentative_state.deviated_motion_pos_fitting = new_pos

    # update part stock
    tentative_state.part_stock[fit_diff] -= 1  # reduce pipe stock
    tentative_state.part_stock[0] -= 1  # reduce fitting stock

    # if any of the following pos is None, there was an error
    pipe_start_pos = None
    pipe_end_pos = None

    # update state grid
    construction_trail = []

    #set construction_parts fittings
    construction_parts = {pos: 0, new_pos: 0}

    for i in range(fit_diff + 1):
        state_pos = (pos[0] + fit_dir[0] * i, pos[1] + fit_dir[1] * i)
        tentative_state.state_grid[state_pos] = 2
        construction_trail.append(state_pos)

        # save info about pipes for later
        if i == 1:
            pipe_start_pos = state_pos
        elif i == fit_diff:
            pipe_end_pos = state_pos
        elif i != 0 and i != fit_diff+1:
            construction_parts[state_pos] = fit_diff
            # add to construction_parts


    if pipe_start_pos is None or pipe_end_pos is None:
        print("Error Code XX")

    # reduce picked parts counter
    tentative_state.picked_parts[0] -= 1
    tentative_state.picked_parts[fit_diff] -= 1

    # add to construction trails
    tentative_state.construction_layouts[construction_trail] = 2
    #todo: use construction trail to remove all motion sets

    # create new layout

    new_construction_path = DefinitePath()

    new_construction_path.append((new_pos, None))
    new_construction_path.append(pipe_start_pos, 0)
    new_construction_path.append(pipe_end_pos, fit_diff)
    new_construction_path.append(pos, 0)

    new_connection =  (new_pos, pos)

    #add to layouts
    tentative_state.layouts.add(new_construction_path)
    #add to connections
    tentative_state.fc_set.add(new_connection)
    #add to construction_parts
    tentative_state.construction_parts.update(construction_parts)
    if new_connection in tentative_state.removed_fc_set:
        tentative_state.removed_fc_set.remove(new_connection)

    return tentative_state