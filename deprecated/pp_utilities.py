def get_updated_state_on_construction_event(tentative_state : State, fit_diff, fit_dir, new_pos, pos) -> State:
    # create new state

    # update fitting_connections
    tentative_state.connection_count[new_pos] = 1
    tentative_state.connection_count[pos] += 1

    # update fitting_pos
    tentative_state.deviated_motion_set_fitting = new_pos

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
    tentative_state.building_instructions[construction_trail] = 2
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
