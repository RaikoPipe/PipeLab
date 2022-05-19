import numpy
import ttkbootstrap
from copy import deepcopy
from idlelib.tooltip import Hovertip
from typing import Optional, Union

import numpy as np
import ttkbootstrap as ttk

from function_test.app_config import button_height
from function_test.app_config import free_style, pipe_style, obs_style, att_style, fit_style, start_style, goal_style, \
    transition_style, fit_deviated_style, att_deviated_style, pipe_deviated_style, fit_misplaced_style, \
    att_misplaced_style, pipe_misplaced_style, start_warning_style, start_success_style, goal_success_style, \
    goal_warning_style, message_error_color, message_deviated_assembly_color, message_conformal_assembly_color, \
    message_construction_complete_color, message_detour_event_color, message_action_undone_color
from path_finding.pf_data_class.path_problem import PathProblem
from process_planning.pp_data_class.construction_state import ConstructionState
from process_planning.pp_data_class.pick_event_result import PickEventResult
from process_planning.pp_data_class.assembly_event_result import AssemblyEventResult
from process_planning.pp_data_class.process_output import ProcessOutput
from process_planning.process_planner import ProcessPlanner
from process_planning.process_state import ProcessState
from type_dictionary import constants
from type_dictionary.type_aliases import StateGrid, Pos, NodeTrail

message_count = 0


def send_new_assembly_event(pos:Pos, event_code:int, process_planner: ProcessPlanner, button_grid:numpy.ndarray,
                            process_message_tree: ttk.Treeview,
                            style_grid: numpy.ndarray,
                            initial_style_grid: numpy.ndarray, part_stock_tree: ttk.Treeview,
                            solution_button_grid: numpy.ndarray, tool_tip_text_grid: numpy.ndarray,
                            update_self_periodically):
    """Sends new pick event to :class:`~process_planner.ProcessPlanner` instance of :class:`~app.FunctionTestApp`.

    Args:
        pos(:obj:`~type_aliases.Pos`): Node position of the motion event.
        event_code(:obj:`int`): Type of motion event. See :ref:`Motion Event Codes`
        process_planner(:class:`~process_planner.ProcessPlanner`): Process planner instance of the function test app
        process_message_tree(:obj:`ttk.Treeview`): Process message display view of the function test app
        part_stock_tree(:obj:`ttk.Treeview`): Part stock display view of the function test app
        button_grid(:obj:`numpy.ndarray`): Grid array containing buttons.
        style_grid(:obj:`numpy.ndarray`): Style grid to overwrite visualization configuration.
        tool_tip_text_grid(:obj:`numpy.ndarray`): Style grid to overwrite tool tip texts.
        solution_button_grid(:obj:`ttk.Treeview`): Button grid with solution layout visualization.

    """
    output = process_planner.handle_motion_event((pos, event_code), handle_detour_events=True,
                                                 ignore_part_check=False)
    # pprint.pprint(output)

    process_state = output.process_state
    event_result: AssemblyEventResult = process_state.last_assembly_event_result
    update_button_grid(button_grid, process_planner.last_process_state, style_grid, tool_tip_text_grid)

    if event_result.detour_event or process_state.detour_trails:
        update_solution_grid(process_state=process_state, solution_button_grid=solution_button_grid,
                             style_grid=initial_style_grid)
    if not update_self_periodically:
        update_trees_on_assembly_event(part_stock_tree, process_message_tree, output)


def send_new_pick_event(part_id: int, process_planner: ProcessPlanner, process_message_tree: ttk.Treeview, part_stock_tree: ttk.Treeview, update_self_periodically):
    """Sends new pick event to :class:`~process_planner.ProcessPlanner` instance of :class:`~app.FunctionTestApp`.

    Args:
        part_id(:obj:`int`): Part ID to pick.
        process_planner(:class:`~process_planner.ProcessPlanner`): Process planner instance of the function test app
        process_message_tree(:obj:`ttk.Treeview`): Process message display view of the function test app
        part_stock_tree(:obj:`ttk.Treeview`): Part stock display view of the function test app


    """
    global message_count
    output = process_planner.handle_motion_event((part_id, 4))

    if not update_self_periodically:
        update_trees_on_pick_event(output, part_id, part_stock_tree, process_message_tree, process_planner)


def update_trees_on_pick_event(output: ProcessOutput, part_id:int, part_stock_tree: ttk.Treeview, process_message_tree: ttk.Treeview, process_planner:ProcessPlanner):
    """Updates the given process message treeview and part stock treeview instance with the updated data in process_output.

    Args:
        output:
        part_id(:obj:`int`): See :paramref:`~send_new_pick_event.part_id`
        process_planner(:class:`~process_planner.ProcessPlanner`): See :paramref:`~send_new_pick_event.process_planner`
        process_message_tree(:obj:`ttk.Treeview`): See :paramref:`~send_new_pick_event.process_message_tree`
        part_stock_tree(:obj:`ttk.Treeview`): See :paramref:`~send_new_pick_event.part_stock_tree`
        """
    global message_count
    event_result: PickEventResult = output.current_event_result
    # pprint.pprint(output)
    message = output.messages[0]
    special_message = output.messages[1]
    process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count,
                                text=message.replace("Process Planner: ", ""))
    if event_result.error:
        process_message_tree.tag_configure(tagname=message_count, background=message_error_color, foreground="white")
    else:
        process_message_tree.tag_configure(tagname=message_count, background=message_conformal_assembly_color,
                                           foreground="black")
    extra_message_ids = [message_count]
    message_count += 1
    if special_message:
        process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count, text=special_message)
        extra_message_ids.append(message_count)
        message_count += 1
    append_attributes_to_tree_entry(event_result, extra_message_ids, process_message_tree)
    item = part_stock_tree.get_children()[part_id]
    part_stock_tree.item(item, values=(part_id, process_planner.last_process_state.picked_parts.count(part_id),
                                       process_planner.last_process_state.part_stock[part_id]))
    process_message_tree.yview_moveto(1)


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
    process_message_tree.tag_configure(tagname=message_count, background=message_action_undone_color, foreground="black")
    message_count += 1
    process_state = process_planner.last_process_state

    update_solution_grid(process_state=process_state, solution_button_grid=solution_button_grid,
                         style_grid=initial_style_grid)

    process_message_tree.yview_moveto(1)


def update_button_grid(button_grid: np.ndarray, process_state: ProcessState, style_grid: np.ndarray,
                       tool_tip_text_grid: np.ndarray):
    """Updates buttons in :paramref:`button_grid` after updating :paramref:`style_grid` and
    :paramref:`tool_tip_text_grid`. Shape of each array must match.

    Args:
        button_grid(:obj:`numpy.ndarray`): Grid array containing buttons.
        process_state(:class:`~process_planning.process_state.ProcessState`): Process state that was modified.
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


def update_style_grid(process_state, style_grid: np.ndarray) -> np.ndarray:
    """Reads the :obj:`process_planner.ProcessPlanner.motion_dict` from the process planner and updates the visualization
    configuration of the :paramref:`style_grid``.

    Args:
        process_state(:class:`~process_state.ProcessState`): Process state that was modified.
        style_grid(:obj:`numpy.ndarray`): See :paramref:`~update_button_grid.style_grid`
    Returns:
        Style grid (:obj:`numpy.ndarray`) with modified visualization configuration.
    """

    start = process_state.aimed_solution.path_problem.start_pos
    goal = process_state.aimed_solution.path_problem.goal_pos

    # reset style grid
    for pos, state in np.ndenumerate(process_state.aimed_solution.path_problem.state_grid):
        if state == constants.free_state:
            style_grid[pos] = free_style
        elif state == constants.obstacle_state:
            style_grid[pos] = obs_style
        elif state == constants.transition_state:
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


def get_style_from_construction_state(construction_state: Optional[ConstructionState]) -> str:
    """Evaluates which style configuration should used for the given parameter.

    Args:
        construction_state(:class:`~construction_state.ConstructionState`)

    """
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


def update_tool_tip_text_grid(process_state: ProcessState, tool_tip_text_grid: np.ndarray) -> np.ndarray:
    """Reads the :obj:`~type_aliases.MotionDict` from the process state and updates each items
    attributes as string in the :paramref:`tool_tip_text_grid`.

    Args:
        process_state(:class:`~process_state.ProcessState`): Process state that was modified.
        tool_tip_text_grid(:obj:`numpy.ndarray`): See :paramref:`~update_button_grid.tool_tip_text_grid`
    Returns:
        Tool tip text grid (:obj:`numpy.ndarray`) with modified texts.
    """
    # reset
    for pos, state in np.ndenumerate(process_state.aimed_solution.path_problem.state_grid):
        if state == constants.free_state:
            tool_tip_text_grid[pos] = str(pos) + "\n" + "free"
        elif state == constants.obstacle_state:
            tool_tip_text_grid[pos] = str(pos) + "\n" + "obstructed"
        elif state == constants.transition_state:
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


def get_construction_state_attributes_as_string(construction_state: ConstructionState) -> str:
    """Reads variable names and values from construction_state and returns them as a formatted string.

    Args:
        construction_state(:class:`~construction_state.ConstructionState`): Construction state to read.

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


def update_solution_grid(process_state: ProcessState, solution_button_grid: np.ndarray, style_grid: np.ndarray):
    """Updates buttons in :paramref:`button_grid` after updating :paramref:`style_grid` and
    :paramref:`tool_tip_text_grid`. Shape of each array must match.

    Args:
        solution_button_grid(:obj:`numpy.ndarray`): Grid array containing buttons.
        process_state(:class:`~process_planning.process_state.ProcessState`): Process state with an updated solution.
        style_grid(:obj:`numpy.ndarray`): Style grid with initial configuration.
        """
    # reset
    for pos, style in np.ndenumerate(style_grid):
        if style != fit_style or style != pipe_style:
            solution_button_grid[pos].configure(style=style)

    for pos, part_id in process_state.aimed_solution.node_trail.items():
        if part_id == 0:
            solution_button_grid[pos].configure(style=fit_style)
        else:
            solution_button_grid[pos].configure(style=pipe_style)

    start = process_state.aimed_solution.path_problem.start_pos
    goal = process_state.aimed_solution.path_problem.goal_pos

    solution_button_grid[start].configure(style=start_success_style)
    solution_button_grid[goal].configure(style=goal_success_style)


def update_trees_on_assembly_event(part_stock_tree: ttk.Treeview, process_message_tree: ttk.Treeview, process_output: ProcessOutput):
    """Updates the given process message treeview and part stock treeview instance with the updated data in process_output.

    Args:

        process_output(:class:`~process_output.ProcessOutput`): Output by the process planner.
        process_message_tree(:obj:`ttk.Treeview`): Process message display view of the function test app
        part_stock_tree(:obj:`ttk.Treeview`): Part stock display view of the function test app
"""

    process_state = process_output.process_state
    messages = process_output.messages
    next_recommended_action = process_output.next_recommended_action
    event_result = process_output.current_event_result

    message = messages[0]
    special_message = messages[1]
    detour_message = messages[2]
    global message_count
    if message:
        process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count,
                                    text=message.replace("Process Planner: ", ""))
        if any((event_result.deviated, event_result.unnecessary, event_result.misplaced)):
            if not event_result.removal:
                process_message_tree.tag_configure(tagname=message_count, background=message_deviated_assembly_color, foreground="black")
        else:
            process_message_tree.tag_configure(tagname=message_count, background=message_conformal_assembly_color, foreground="black")

        if event_result.error:
            process_message_tree.tag_configure(tagname=message_count, background=message_error_color, foreground="white")

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

        append_attributes_to_tree_entry(event_result, extra_message_ids, process_message_tree)
    if detour_message:
        process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count, text=detour_message)
        process_message_tree.tag_configure(tagname=message_count, background=message_detour_event_color, foreground="black")
        message_count += 1
    if process_state.completion == 1:
        process_message_tree.insert("", index=ttk.END, tag=message_count, iid=message_count,
                                    text="Construction complete!")
        process_message_tree.tag_configure(tagname=message_count, background=message_construction_complete_color, foreground="black")
        message_count += 1

    for p_id in process_state.part_stock.keys():
        item = part_stock_tree.get_children()[p_id]
        part_stock_tree.item(item, values=(p_id, process_state.picked_parts.count(p_id),
                                           process_state.part_stock[p_id]))
    process_message_tree.yview_moveto(1)


def append_attributes_to_tree_entry(event_result:Union[AssemblyEventResult, PickEventResult], extra_message_ids:list, process_message_tree: ttk.Treeview):
    """Inserts attributes in event_info to the given process message treeview and appends them to the parent
    entry specified in the first list entry of extra_message_ids.

    Args:
        event_result(:obj:`Union` [:class:`~assembly_event_result.AssemblyEventResult`, :class:`~pick_event_result.PickEventResult`])
        extra_message_ids(:obj`list`): List containing IDs of entries to add to a parent entry. First entry in the list must
        be the ID of the parent entry.
        process_message_tree(:obj:`ttk.Treeview`): Process message treeview of the app.

    """
    global message_count
    for attribute in vars(event_result):
        value = event_result.__getattribute__(attribute)
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


def get_button_grid(state_grid: StateGrid, node_trail: NodeTrail, button_grid_frame: ttk.LabelFrame,
                    process_planner: Optional[ProcessPlanner],
                    part_select_option: Optional[ttk.IntVar], process_message_tree: Optional[ttk.Treeview],
                    part_stock_tree: Optional[ttk.Treeview], solution_button_grid: Optional[np.ndarray], path_problem: PathProblem,
                    update_self_periodically: bool = False):
    """Constructs either a static or interactive button grid, depending on if a process planner was given or not.
    Also creates a style grid and a tool tip text grid for saving visualization information.

    Args:
        path_problem(:class:`~path_problem.PathProblem`):
        solution_button_grid(:obj:`Optional` [:obj:`numpy.ndarray`]): Needed for making button grid interactive.
        process_planner(:obj:`Optional` [:obj:`~process_planner.ProcessPlanner`]): If given, makes button grid interactive.
        button_grid_frame(:obj:`ttk.LabelFrame`): LabelFrame object to put the button grid in.
        state_grid(:obj:`~type_aliases.StateGrid`): Used for determining the initial visualization configuration
        node_trail(:obj:`~type_aliases.NodeTrail`): Used for determining what parts were used in the solution.
        part_select_option(:obj:`Optional` [:obj:`ttk.IntVar`]): Option to communicate which part was selected on inputting a motion event.
        process_message_tree(:obj:`Optional` [:obj:`ttk.Treeview`]): Process message display view of the function test app
        part_stock_tree(:obj:`Optional` [:obj:`ttk.Treeview`]): Part stock display view of the function test app

    Returns:
        (:obj:`numpy.ndarray`) of button grid, style grid and tool tip text grid with matching shapes.

    """
    start = path_problem.start_pos
    goal = path_problem.goal_pos

    # Grid Button
    button_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=ttk.Button)
    style_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=np.dtype("U100"))
    tool_tip_text_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=np.dtype("U100"))
    for pos, state in np.ndenumerate(state_grid):
        style = ""
        if state == constants.free_state:
            style = free_style
            style_grid[pos] = free_style
            tool_tip_text_grid[pos] = str(pos) + "\n" + "free"
        elif state == constants.obstacle_state:
            style = obs_style
            style_grid[pos] = obs_style
            tool_tip_text_grid[pos] = str(pos) + "\n" + "obstructed"
        elif state == constants.part_state:
            # what part was used?
            if node_trail[pos] == 0:
                style = fit_style
                style_grid[pos] = fit_style
            else:
                style = pipe_style
                style_grid[pos] = pipe_style
        elif state == constants.transition_state:
            style = transition_style
            style_grid[pos] = transition_style
            tool_tip_text_grid[pos] = str(pos) + "\n" + "Transition"

        if pos == start:
            if not process_planner:
                style = start_success_style
            else:
                style = start_style
                style_grid[pos] = start_style
        elif pos == goal:
            if not process_planner:
                style = goal_success_style
            else:
                style = goal_style
                style_grid[pos] = goal_style

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
            command = lambda t=pos: send_new_assembly_event(t, part_select_option.get(), process_planner, button_grid,
                                                            process_message_tree, style_grid, initial_style_grid,
                                                            part_stock_tree=part_stock_tree,
                                                            solution_button_grid=solution_button_grid,
                                                            tool_tip_text_grid=tool_tip_text_grid,
                                                            update_self_periodically= update_self_periodically)
            button.config(command=command)



        else:
            button = ttk.Button(button_grid_frame, text=str(pos), style=style)

        button.grid(row=pos[0], column=pos[1], ipady=button_height, padx=0, ipadx=0, sticky="nsew")
        button_grid[pos] = button

    # previous_style_grids.insert(0, deepcopy(style_grid))

    return button_grid, style_grid, tool_tip_text_grid


initial_style_grid = None
