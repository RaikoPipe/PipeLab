from copy import deepcopy
from idlelib.tooltip import Hovertip
from typing import Optional
from function_test.app_config import GridButton, button_width, button_height
import ttkbootstrap as ttk
from ttkbootstrap.constants import *

import numpy as np

from function_test.app_config import free_style, pipe_style, obs_style, att_style, fit_style, start_style, goal_style, \
    transition_style, fit_deviated_style, att_deviated_style, pipe_deviated_style, fit_misplaced_style, \
    att_misplaced_style, pipe_misplaced_style, start_warning_style, start_success_style, goal_success_style, \
    goal_warning_style
from process_planning.pp_data_class.construction_state import ConstructionState
from process_planning.pp_data_class.pick_event_info import PickEventInfo
from process_planning.pp_data_class.placement_event_info import PlacementEventInfo
from process_planning.process_planner import ProcessPlanner
from process_planning.process_state import ProcessState
from type_dictionary.type_aliases import StateGrid

message_count = 0


def get_style_from_construction_state(construction_state: Optional[ConstructionState]):
    style = free_style
    if construction_state:
        if construction_state.event_code == 3:
            if construction_state.unnecessary or construction_state.deviated:
                style = att_deviated_style
            elif construction_state.misplaced:
                style = att_misplaced_style
            else:
                style = att_style
        elif construction_state.event_code == 2:
            if construction_state.unnecessary or construction_state.deviated:
                style = pipe_deviated_style
            elif construction_state.misplaced:
                style = pipe_misplaced_style
            else:
                style = pipe_style
        elif construction_state.event_code == 1:
            if construction_state.unnecessary or construction_state.deviated:
                style = fit_deviated_style
            elif construction_state.misplaced:
                style = fit_misplaced_style
            else:
                style = fit_style

    return style


def update_style_grid(process_state, style_grid: np.ndarray) -> np.ndarray:
    """Reads the :obj:`process_planner.ProcessPlanner.motion_dict` from the process planner and updates the visualization
    configuration of the :paramref:`style_grid``.

    Args:
        process_state(:class:`process_planning.process_state.ProcessState`): Process state that was modified.
        style_grid(:obj:`numpy.ndarray`): See :paramref:`~update_button_grid.style_grid`
    Returns:
        Style grid (:obj:`numpy.ndarray`) with modified visualization configuration.
    """

    start = process_state.aimed_solution.path_problem.start_pos
    goal = process_state.aimed_solution.path_problem.goal_pos

    # reset style grid
    for pos, state in np.ndenumerate(process_state.aimed_solution.path_problem.state_grid):
        if state == 0:
            style_grid[pos] = free_style
        elif state == 1:
            style_grid[pos] = obs_style
        elif state == 3:
            style_grid[pos] = transition_style
        else:
            style_grid[pos] = free_style

    # set style grid
    for motion_pos, construction_state in process_state.motion_dict.items():
        if not isinstance(motion_pos[0], int):
            for pos in motion_pos:
                style = get_style_from_construction_state(construction_state)
                style_grid[pos] = style
        else:
            style = get_style_from_construction_state(construction_state)
            style_grid[motion_pos] = style

    sgd_dict = {start: start_warning_style, goal: goal_warning_style}
    sgs_dict = {start: start_success_style, goal: goal_success_style}
    sgf_dict = {start: start_style, goal: goal_style}

    for pos in (start, goal):
        if process_state.motion_dict.get(pos):
            construction_state = process_state.motion_dict[pos]
            if construction_state.deviated:
                style_grid[pos] = sgd_dict[pos]
            else:
                style_grid[pos] = sgs_dict[pos]
        else:
            style_grid[pos] = sgf_dict[pos]

    return style_grid


def get_construction_state_attributes_as_string(construction_state: ConstructionState) -> str:
    """Reads variable names and values from :class:`pp_data_class.construction_state.ConstructionState` and returns them
     as a formatted string.

    Args:
        construction_state(:class:`pp_data_class.construction_state.ConstructionState`): Construction state to read.

    Returns:
        :obj:`str`

    """
    text = ""
    for attribute in vars(construction_state):
        value = construction_state.__getattribute__(attribute)
        if attribute == "time_registered":
            value = str(value.replace(microsecond=0))
        elif attribute == "part_id":
            value = str(value)
        if value:
            text += attribute + ": " + str(value) + "\n"

    return text


def update_tool_tip_text_grid(process_state: ProcessState, tool_tip_text_grid: np.ndarray) -> np.ndarray:
    """Reads the :obj:`process_planner.ProcessPlanner.motion_dict` from the process planner and updates each items
    attributes as string in the :paramref:`tool_tip_text_grid`.

    Args:
        process_state(:class:`~process_planning.process_state.ProcessState`): Process state that was modified.
        tool_tip_text_grid(:obj:`numpy.ndarray`): See :paramref:`~update_button_grid.tool_tip_text_grid`
    Returns:
        Tool tip text grid (:obj:`numpy.ndarray`) with modified texts.
    """
    # reset
    for pos, state in np.ndenumerate(process_state.aimed_solution.path_problem.state_grid):
        if state == 0:
            tool_tip_text_grid[pos] = str(pos) + "\n" + "free"
        elif state == 1:
            tool_tip_text_grid[pos] = str(pos) + "\n" + "obstructed"
        elif state == 3:
            tool_tip_text_grid[pos] = str(pos) + "\n" + "Transition"

        part_id = process_state.aimed_solution.node_trail.get(pos)
        if part_id is not None:
            text = str.format(str(pos) + "\n" + f"Required part ID: {part_id}")
            tool_tip_text_grid[pos] = text

    # set
    for motion_pos, construction_state in process_state.motion_dict.items():
        if not isinstance(motion_pos[0], int):
            for pos in motion_pos:
                text = get_construction_state_attributes_as_string(construction_state)

                tool_tip_text_grid[pos] = text
        else:
            text = get_construction_state_attributes_as_string(construction_state)

            tool_tip_text_grid[motion_pos] = text

    return tool_tip_text_grid

    pass


def update_button_grid(button_grid: np.ndarray, process_state: ProcessState, style_grid: np.ndarray,
                       tool_tip_text_grid: np.ndarray):
    """Updates buttons in :paramref:`button_grid` after updating :paramref:`style_grid` and
    :paramref:`tool_tip_text_grid`. Shape of each array must match.

    Args:
        button_grid(:obj:`numpy.ndarray`): Grid array containing buttons.
        process_state(:class:`~process_planning.process_state.ProcessState`)
        style_grid(:obj:`numpy.ndarray`): Style grid to overwrite visualization configuration.
        tool_tip_text_grid(:obj:`numpy.ndarray`): Style grid to overwrite tool tip texts.
        """

    style_grid = update_style_grid(process_state, style_grid)
    tool_tip_text_grid = update_tool_tip_text_grid(process_state, tool_tip_text_grid)

    # update button grid
    for pos, style in np.ndenumerate(style_grid):
        button_grid[pos].configure(style=style)

    # update hovertips
    for pos, tool_tip_text in np.ndenumerate(tool_tip_text_grid):
        Hovertip(button_grid[pos], tool_tip_text, hover_delay=0)

    # reset button text
    for pos, _ in np.ndenumerate(process_state.state_grid):
        button_grid[pos].configure(text=str(pos))

    # update button text
    for _, construction_state in process_state.motion_dict.items():
        construction_state: ConstructionState
        button = button_grid[construction_state.event_pos]
        button: ttk.Button
        button_text = button.cget("text")
        button_text: str
        button_text.replace("?", "")
        if construction_state.part_id == -2:
            button_text += "?"

        button_grid[construction_state.event_pos].configure(text=button_text)


def undo_action(process_planner: ProcessPlanner, button_grid: np.ndarray, style_grid: np.ndarray,
                part_stock_tree: ttk.Treeview, tool_tip_text_grid, process_message_tree: ttk.Treeview,
                solution_button_grid: np.ndarray):
    """Undoes the last action by reverting to the previous process state.

    Args:

        solution_button_grid(:obj:`ttk.Treeview`): Button grid with solution layout visualization.
        process_message_tree(:obj:`ttk.Treeview`): Process message display.
        part_stock_tree: Part stock display.
        button_grid(:obj:`numpy.ndarray`): Grid array containing visualization button for the current process state.
        process_planner(:class:`~process_planning.process_planner.ProcessPlanner`): The latest process planner.
        style_grid(:obj:`numpy.ndarray`): Style grid to overwrite visualization configuration.
        tool_tip_text_grid(:obj:`numpy.ndarray`): Style grid to overwrite tool tip texts.

    """
    process_planner.return_to_previous_state()
    update_button_grid(button_grid=button_grid, process_state=process_planner.last_process_state,
                       tool_tip_text_grid=tool_tip_text_grid,
                       style_grid=style_grid)

    part_id = 0
    for item in part_stock_tree.get_children():
        part_stock_tree.item(item, values=(part_id, process_planner.last_process_state.picked_parts.count(part_id),
                                           process_planner.last_process_state.part_stock[part_id]))
        part_id += 1

    global message_count
    process_message_tree.insert("", index=ttk.END, tag=message_count, text="Last Action was undone!")
    process_message_tree.tag_configure(tagname=message_count, background="brown1", foreground="black")
    message_count += 1
    process_state = process_planner.last_process_state

    update_solution_grid(tentative_state=process_state, solution_button_grid=solution_button_grid,
                         initial_style_grid=initial_style_grid)

    process_message_tree.yview_moveto(1)


def update_solution_grid(tentative_state: ProcessState, solution_button_grid, initial_style_grid: np.ndarray):
    """Updates buttons in :paramref:`button_grid` after updating :paramref:`style_grid` and
    :paramref:`tool_tip_text_grid`. Shape of each array must match.

    Args:
        solution_button_grid(:obj:`numpy.ndarray`): Grid array containing buttons.
        process_state(:class:`~process_planning.process_state.ProcessState`)
        style_grid(:obj:`numpy.ndarray`): Style grid to overwrite visualization configuration.
        tool_tip_text_grid(:obj:`numpy.ndarray`): Style grid to overwrite tool tip texts.
        """
    #
    for pos, style in np.ndenumerate(initial_style_grid):
        if style != fit_style or style != pipe_style:
            solution_button_grid[pos].configure(style=style)

    for pos, part_id in tentative_state.aimed_solution.node_trail.items():
        if part_id == 0:
            solution_button_grid[pos].configure(style=fit_style)
        else:
            solution_button_grid[pos].configure(style=pipe_style)


def send_new_placement_event(pos, event_code, process_planner: ProcessPlanner, button_grid, process_message_tree,
                             style_grid,
                             initial_style_grid, part_stock_tree, solution_button_grid, tool_tip_text_grid, root=None):
    output = process_planner.handle_motion_event((pos, event_code), handle_detour_events=True,
                                                 ignore_part_check=False)
    # pprint.pprint(output)
    process_state = output.process_state
    messages = output.messages
    next_recommended_action = output.next_recommended_action
    event_info: PlacementEventInfo = process_state.last_placement_event_info
    update_button_grid(button_grid, process_planner.last_process_state, style_grid, tool_tip_text_grid)

    if event_info.detour_event or process_state.detour_trails:
        update_solution_grid(tentative_state=process_state, solution_button_grid=solution_button_grid,
                             initial_style_grid=initial_style_grid)

    update_process_message_tree(event_info, messages, next_recommended_action, part_stock_tree, process_message_tree,
                                process_planner)


def update_process_message_tree(event_info, messages, next_recommended_action, part_stock_tree, process_message_tree,
                                process_planner):
    message = messages[0]
    special_message = messages[1]
    detour_message = messages[2]
    global message_count
    if message:
        process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count,
                                    text=message.replace("Process Planner: ", ""))
        if any((event_info.deviated, event_info.unnecessary, event_info.misplaced)):
            if not event_info.removal:
                process_message_tree.tag_configure(tagname=message_count, background="yellow2", foreground="black")
        else:
            process_message_tree.tag_configure(tagname=message_count, background="green2", foreground="black")

        if event_info.error:
            process_message_tree.tag_configure(tagname=message_count, background="red3", foreground="white")

        extra_message_ids = [message_count]
        message_count += 1

        if special_message:
            process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count, text=special_message)
            extra_message_ids.append(message_count)
            message_count += 1

        if next_recommended_action:
            process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count,
                                        text="Next recommended action: " + str(next_recommended_action))
            extra_message_ids.append(message_count)
            message_count += 1

        append_texts_to_message(event_info, extra_message_ids, process_message_tree)
    if detour_message:
        process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count, text=detour_message)
        process_message_tree.tag_configure(tagname=message_count, background="maroon1", foreground="black")
        message_count += 1
    if process_planner.last_process_state.completion == 1:
        process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count,
                                    text="Construction complete!")
        process_message_tree.tag_configure(tagname=message_count, background="gold", foreground="black")
        message_count += 1
    for p_id in process_planner.last_process_state.part_stock.keys():
        item = part_stock_tree.get_children()[p_id]
        part_stock_tree.item(item, values=(p_id, process_planner.last_process_state.picked_parts.count(p_id),
                                           process_planner.last_process_state.part_stock[p_id]))
    process_message_tree.yview_moveto(1)


def append_texts_to_message(event_info, extra_message_ids, process_message_tree):
    global message_count
    for attribute in vars(event_info):
        value = event_info.__getattribute__(attribute)
        if attribute == "detour_event" and value:
            value = True
        elif attribute == "time_registered":
            value = str(value.replace(microsecond=0))
        elif attribute == "part_id":
            value = str(value)
        elif attribute in {"completed_layouts", "stock_empty"}:
            value = tuple(value)  # tkinter gets key error on sets
        if value:
            value = str(value)
            process_message_tree.insert("", index=ttk.END, iid=message_count, tag=message_count,
                                        text=str.format(f"{attribute}: {value}"))
            extra_message_ids.append(message_count)
            message_count += 1
    parent_message_id = extra_message_ids[0]
    for idx, child_message_id in enumerate(extra_message_ids):
        if idx == 0:
            continue
        process_message_tree.move(child_message_id, parent_message_id, idx - 1)


def send_new_pick_event(part_id, process_planner: ProcessPlanner, process_message_tree, part_stock_tree):
    """Sends new pick event to process planner."""
    global message_count
    output = process_planner.handle_motion_event((part_id, 4))
    event_info: PickEventInfo = output.current_event_info
    # pprint.pprint(output)
    message = output.messages[0]
    special_message = output.messages[1]

    process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count,
                                text=message.replace("Process Planner: ", ""))

    if event_info.error:
        process_message_tree.tag_configure(tagname=message_count, background="red3", foreground="white")
    else:
        process_message_tree.tag_configure(tagname=message_count, background="green2", foreground="black")

    extra_message_ids = [message_count]
    message_count += 1

    if special_message:
        process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count, text=special_message)
        extra_message_ids.append(message_count)
        message_count += 1

    append_texts_to_message(event_info, extra_message_ids, process_message_tree)

    item = part_stock_tree.get_children()[part_id]
    part_stock_tree.item(item, values=(part_id, process_planner.last_process_state.picked_parts.count(part_id),
                                       process_planner.last_process_state.part_stock[part_id]))

    process_message_tree.yview_moveto(1)


def get_button_grid(state_grid: StateGrid, absolute_trail, start, goal, button_grid_frame,
                    process_planner: Optional[ProcessPlanner],
                    part_select_option: Optional[ttk.IntVar], tree, part_stock_tree, solution_button_grid):
    """Constructs a button grid."""
    # Grid Button
    button_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=ttk.Button)
    style_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=np.dtype("U100"))
    tool_tip_text_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=np.dtype("U100"))
    for pos, state in np.ndenumerate(state_grid):
        style = ""
        if state == 0:
            style = free_style
            style_grid[pos] = free_style
            tool_tip_text_grid[pos] = str(pos) + "\n" + "free"
        elif state == 1:
            style = obs_style
            style_grid[pos] = obs_style
            tool_tip_text_grid[pos] = str(pos) + "\n" + "obstructed"
        elif state == 2:
            # what part?
            if absolute_trail[pos] == 0:
                style = fit_style
                style_grid[pos] = fit_style
            else:
                style = pipe_style
                style_grid[pos] = pipe_style
        elif state == 3:
            style = transition_style
            style_grid[pos] = transition_style
            tool_tip_text_grid[pos] = str(pos) + "\n" + "Transition"

        if pos == start:
            style = start_style
            style_grid[pos] = start_style
        elif pos == goal:
            style_grid[pos] = start_style
            style = goal_style
        global initial_style_grid
        initial_style_grid = deepcopy(style_grid)

        if process_planner:
            button = ttk.Button(button_grid_frame, text=str(pos), style=style)
            part_id = process_planner.initial_process_state.aimed_solution.node_trail.get(pos)
            if part_id is not None:
                text = str.format(str(pos) + "\n" + f"Required part ID: {part_id}")
                tool_tip_text_grid[pos] = text
                Hovertip(button, text, hover_delay=0)
            else:
                Hovertip(button, tool_tip_text_grid[pos], hover_delay=0)
            command = lambda t=pos: send_new_placement_event(t, part_select_option.get(), process_planner, button_grid,
                                                             tree, style_grid, initial_style_grid,
                                                             part_stock_tree=part_stock_tree,
                                                             solution_button_grid=solution_button_grid,
                                                             tool_tip_text_grid=tool_tip_text_grid)
            button.config(command=command)



        else:
            button = ttk.Button(button_grid_frame, text=str(pos), style=style)

        button.grid(row=pos[0], column=pos[1], ipady=button_height, padx=0, ipadx=0, sticky="nsew")
        button_grid[pos] = button

    # previous_style_grids.insert(0, deepcopy(style_grid))

    return button_grid, style_grid, tool_tip_text_grid


initial_style_grid = np.ndarray
