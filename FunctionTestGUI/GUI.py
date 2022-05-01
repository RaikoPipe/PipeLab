import sys
import tkinter as tk
from copy import deepcopy
from idlelib.tooltip import Hovertip
from tkinter import ttk
from typing import Optional
import pathlib
import numpy as np

from ProcessPlanning.classes.data_class.ConstructionState import ConstructionState
from ProcessPlanning.classes.data_class.EventInfo import EventInfo
from PathFinding.data_class.PathProblem import PathProblem
from ProcessPlanning.ProcessPlanner import ProcessPlanner
from ProcessPlanning.classes.ProcessState import ProcessState
from ProcessPlanning.util.pp_util import get_absolute_trail_from_building_instructions
from os import system

free_style = "FREE.TButton"
pipe_style = "PIPE.TButton"
obs_style = "OBSTACLE.TButton"
att_style = "ATTACHMENT.TButton"
fit_style = "CORNER.TButton"
start_style = "START.TButton"
goal_style = "GOAL.TButton"
transition_style = "TRANSITION.TButton"
fit_deviated_style = "FITDEV.TButton"
att_deviated_style = "ATTDEV.TButton"
pipe_deviated_style = "PIPEDEV.TButton"
fit_misplaced_style = "FITMIS.TButton"
att_misplaced_style = "ATTMIS.TButton"
pipe_misplaced_style = "PIPEMIS.TButton"
highlight_next_rec_action_style = "NEXTRECACT.TButton"

treeview_style = "TREESTYLE.Treeview"


button_dict = {}


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


def get_style_grid(process_planner, style_grid):
    # reset style grid
    for pos, state in np.ndenumerate(process_planner.tentative_process_state.aimed_solution.path_problem.state_grid):
        if state == 0:
            style_grid[pos] = free_style
        elif state == 1:
            style_grid[pos] = obs_style
        elif state == 3:
            style_grid[pos] = transition_style
        else:
            style_grid[pos] = free_style

    # set style grid
    for motion_pos, construction_state in process_planner.tentative_process_state.motion_dict.items():
        if not isinstance(motion_pos[0], int):
            for pos in motion_pos:
                style = get_style_from_construction_state(construction_state)
                style_grid[pos] = style
        else:
            style = get_style_from_construction_state(construction_state)
            style_grid[motion_pos] = style

    return style_grid


def get_attributes_as_string(data_class):
    text = ""
    for attribute in vars(data_class):
        value = data_class.__getattribute__(attribute)
        if attribute == "time_registered":
            value = str(value.replace(microsecond=0))
        elif attribute == "part_id":
            value = str(value)
        if value:
            text += attribute + ": " + str(value) + "\n"

    return text


def get_tool_tip_text_grid(process_planner, tool_tip_text_grid):
    # reset
    for pos, state in np.ndenumerate(process_planner.tentative_process_state.aimed_solution.path_problem.state_grid):
        if state == 0:
            tool_tip_text_grid[pos] = "free"
        elif state == 1:
            tool_tip_text_grid[pos] = "obstructed"
        elif state == 3:
            tool_tip_text_grid[pos] = "Transition"

        part_id = process_planner.tentative_process_state.aimed_solution.absolute_trail.get(pos)
        if part_id is not None:
            text = str.format(f"Required part ID: {part_id}")
            tool_tip_text_grid[pos] = text

    # set
    for motion_pos, construction_state in process_planner.tentative_process_state.motion_dict.items():
        if not isinstance(motion_pos[0], int):
            for pos in motion_pos:
                text = get_attributes_as_string(construction_state)

                tool_tip_text_grid[pos] = text
        else:
            text = get_attributes_as_string(construction_state)

            tool_tip_text_grid[motion_pos] = text

    return tool_tip_text_grid

    pass


def update_button_grid(button_grid, process_planner, style_grid, tool_tip_text_grid):
    # previous_style_grids.insert(0, deepcopy(style_grid))

    style_grid = get_style_grid(process_planner, style_grid)
    tool_tip_text_grid = get_tool_tip_text_grid(process_planner, tool_tip_text_grid)

    style_grid[process_planner.tentative_process_state.aimed_solution.path_problem.start_pos] = start_style
    style_grid[process_planner.tentative_process_state.aimed_solution.path_problem.goal_pos] = goal_style

    # update button grid
    for pos, style in np.ndenumerate(style_grid):
        button_grid[pos].configure(style=style)

    # update tooltip-text grid
    for pos, tool_tip_text in np.ndenumerate(tool_tip_text_grid):
        Hovertip(button_grid[pos], tool_tip_text, hover_delay=0)

    # reset button text
    for pos, _ in np.ndenumerate(process_planner.tentative_process_state.state_grid):
        button_grid[pos].configure(text=str(pos))

    # update button text
    for _, construction_state in process_planner.tentative_process_state.motion_dict.items():
        construction_state: ConstructionState
        button = button_grid[construction_state.event_pos]
        button: ttk.Button
        button_text = button.cget("text")
        button_text: str
        button_text.replace("?", "")
        if construction_state.part_id == -2:
            button_text += "?"

        button_grid[construction_state.event_pos].configure(text=button_text)


def undo_action(process_planner, button_grid, style_grid, part_stock_tree, tool_tip_text_grid, process_message_tree):
    process_planner.return_to_previous_state()
    update_button_grid(button_grid=button_grid, process_planner=process_planner, tool_tip_text_grid=tool_tip_text_grid,
                       style_grid=style_grid)
    # if previous_style_grids:
    #     style_grid = previous_style_grids.pop(0)
    #     for pos, style in np.ndenumerate(style_grid):
    #         button_grid[pos].configure(style=style)
    #
    part_id = 0
    for item in part_stock_tree.get_children():
        part_stock_tree.item(item, values=(part_id, process_planner.tentative_process_state.picked_parts.count(part_id),
                                           process_planner.tentative_process_state.part_stock[part_id]))
        part_id += 1

    global message_count
    process_message_tree.insert("", index=tk.END, text="Last Action was undone!")
    process_message_tree.tag_configure(tagname=message_count, background="cyan")
    message_count += 1

    process_message_tree.yview_moveto(1)


message_count = 0


def update_solution_grid(tentative_state: ProcessState, button_grid, initial_style_grid):
    #
    for pos, style in np.ndenumerate(initial_style_grid):
        if style != fit_style or style != pipe_style:
            button_grid[pos].configure(style=style)

    for pos, part_id in tentative_state.aimed_solution.absolute_trail.items():
        if part_id == 0:
            button_grid[pos].configure(style=fit_style)
        else:
            button_grid[pos].configure(style=pipe_style)

def send_new_placement_event(pos, event_code, process_planner: ProcessPlanner, button_grid, tree, style_grid,
                             initial_style_grid, part_stock_tree, solution_button_grid, tool_tip_text_grid):
    output = process_planner.main((pos, event_code), handle_detour_events=True,
                                                   ignore_part_check=False)
    process_state = output.process_state
    messages = output.messages
    next_recommended_action = output.next_recommended_action
    event_info: EventInfo = process_state.last_event_info
    update_button_grid(button_grid, process_planner, style_grid, tool_tip_text_grid)

    if event_info.detour_event or process_state.detour_trails:
        update_solution_grid(tentative_state=process_state, button_grid=solution_button_grid,
                             initial_style_grid=initial_style_grid)

    message = messages[0]
    special_message = messages[1]
    detour_message = messages[2]

    global message_count
    if message:
        tree.insert("", index=tk.END, tag=message_count, iid=message_count,
                    text=message.replace("Process Planner: ", ""))
        tree.tag_configure(tagname=message_count, background="green2")
        if any((event_info.deviated, event_info.unnecessary, event_info.misplaced)):
            if not event_info.removal:
                tree.tag_configure(tagname=message_count, background="yellow")
            else:
                tree.tag_configure(tagname=message_count, background="green2")
        elif event_info.removal:
            tree.tag_configure(tagname=message_count, background="yellow")
        if event_info.error:
            tree.tag_configure(tagname=message_count, background="red", foreground="white")

        extra_message_ids = [message_count]
        message_count += 1

        if special_message:
            tree.insert("", index=tk.END, tag=message_count, iid=message_count, text=special_message)
            extra_message_ids.append(message_count)
            message_count += 1

        if next_recommended_action:
            tree.insert("", index=tk.END, tag=message_count, iid=message_count, text="Next recommended action: " + str(next_recommended_action))
            extra_message_ids.append(message_count)
            message_count += 1

        for attribute in vars(event_info):
            value = event_info.__getattribute__(attribute)
            if attribute == "detour_event" and value:
                value = True
            elif attribute == "time_registered":
                value = str(value.replace(microsecond=0))
            elif attribute == "part_id":
                value = str(value)
            elif attribute == "completed_layouts":
                value = tuple(value)  # tkinter gets key error on sets
            if value:
                value = str(value)
                tree.insert("", index=tk.END, iid=message_count, tag=message_count,
                            text=str.format(f"{attribute}: {value}"))
                extra_message_ids.append(message_count)
                message_count += 1

        parent_message_id = extra_message_ids[0]
        for idx, child_message_id in enumerate(extra_message_ids):
            if idx == 0:
                continue
            tree.move(child_message_id, parent_message_id, idx - 1)

    if detour_message:
        tree.insert("", index=tk.END, tag=message_count, iid=message_count, text=detour_message)
        tree.tag_configure(tagname=message_count, background="maroon1")
        message_count += 1

        # todo: make tidier information output
        # if event_info.part_id in process_planner.initial_process_state.part_stock:
        #     tree.insert("", index=tk.END, tag=message_count, values=(str.format(f"ID: {event_info.part_id}"),))
        #     extra_message_ids.append(message_count)
        #     message_count += 1
        # if event_info.removal:
        #     tree.insert("", index=tk.END, tag=message_count, values=("Removal",))
        #     extra_message_ids.append(message_count)
        #     message_count += 1
        # if event_info.deviated:
        #     tree.insert("", index=tk.END, tag=message_count, values=("Deviated",))
        #     extra_message_ids.append(message_count)
        #     message_count += 1
        # if event_info.deviated:
        #     tree.insert("", index=tk.END, tag=message_count, values=("Deviated",))
        #     extra_message_ids.append(message_count)
        #     message_count += 1
        # if event_info.unnecessary:
        #     tree.insert("", index=tk.END, tag=message_count, values=("Unnecessary",))
        #     extra_message_ids.append(message_count)
        #     message_count += 1
        # if event_info.misplaced:
        #     tree.insert("", index=tk.END, tag=message_count, values=("Misplaced",))
        #     extra_message_ids.append(message_count)
        #     message_count += 1

    part_id = event_info.part_id
    if process_planner.tentative_process_state.completion == 1:
        tree.insert("", index=tk.END, tag=message_count, iid=message_count, text="Construction complete!")
        tree.tag_configure(tagname=message_count, background="gold")
        message_count += 1


    for p_id in process_planner.tentative_process_state.part_stock.keys():
        item = part_stock_tree.get_children()[p_id]
        part_stock_tree.item(item, values=(p_id, process_planner.tentative_process_state.picked_parts.count(p_id),
                                           process_planner.tentative_process_state.part_stock[p_id]))
    tree.yview_moveto(1)


def send_new_pick_event(part_id, process_planner: ProcessPlanner, tree, part_stock_tree):
    output = process_planner.main((part_id, 4))
    message = output.messages[0]
    tree.insert("", index=tk.END, text=message.replace("util: ", ""))
    item = part_stock_tree.get_children()[part_id]
    part_stock_tree.item(item, values=(part_id, process_planner.tentative_process_state.picked_parts.count(part_id),
                                       process_planner.tentative_process_state.part_stock[part_id]))
    # previous_style_grids.insert(0, deepcopy(style_grid))
    tree.yview_moveto(1)


def get_button_grid(state_grid: np.ndarray, absolute_trail, start, goal, button_grid_frame,
                    process_planner: Optional[ProcessPlanner],
                    part_select_option: Optional[tk.IntVar], tree, part_stock_tree, solution_button_grid):
    # Grid Button
    button_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=ttk.Button)
    style_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=np.dtype("U100"))
    tool_tip_text_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=np.dtype("U100"))
    for pos, state in np.ndenumerate(state_grid):
        style = ""
        if state == 0:
            style = free_style
            style_grid[pos] = free_style
            tool_tip_text_grid[pos] = "free"
        elif state == 1:
            style = obs_style
            style_grid[pos] = obs_style
            tool_tip_text_grid[pos] = "obstructed"
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
            tool_tip_text_grid[pos] = "Transition"

        if pos == start:
            style = start_style
            style_grid[pos] = start_style
        elif pos == goal:
            style_grid[pos] = start_style
            style = goal_style

        initial_style_grid = deepcopy(style_grid)

        if process_planner:
            button = ttk.Button(button_grid_frame, text=str(pos), style=style)
            part_id = process_planner.initial_process_state.aimed_solution.absolute_trail.get(pos)
            if part_id is not None:
                text = str.format(f"Required part ID: {part_id}")
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

        button.grid(row=pos[0], column=pos[1], ipady=8, ipadx=3)
        button_grid[pos] = button

    # previous_style_grids.insert(0, deepcopy(style_grid))

    return button_grid, style_grid, tool_tip_text_grid


class function_test_app:
    def __init__(self, state_grid: np.ndarray, path_problem: PathProblem, initial_state: Optional[ProcessState], use_dark_theme:bool=False):
        root = tk.Tk()

        self.process_planner = ProcessPlanner(initial_path_problem=path_problem, initial_process_state=initial_state)

        start = path_problem.start_pos
        goal = path_problem.goal_pos

        part_select_option = tk.IntVar(value=1)
        pos_hovered = start

        style = ttk.Style()
        button_width = 5
        button_height = 8
        font = ('Tahoma', 7)

        style.configure(free_style, background="white", width=button_width, height=button_height, font=font)
        style.configure(pipe_style, background="blue", width=button_width, height=button_height, font=font)
        style.configure(obs_style, background="orange", width=button_width, height=button_height, font=font)
        style.configure(fit_style, background="cyan", width=button_width, height=button_height, font=font)
        style.configure(start_style, background="green", width=button_width, height=button_height, font=font)
        style.configure(goal_style, background="red", width=button_width, height=button_height, font=font)
        style.configure(transition_style, background="black", width=button_width, height=button_height, font=font)
        style.configure(att_style, background="magenta", width=button_width, height=button_height, font=font)
        style.configure(fit_deviated_style, background="cyan", foreground="yellow", width=button_width,
                        height=button_height,
                        font=font)
        style.configure(att_deviated_style, background="magenta", foreground="yellow", width=button_width,
                        height=button_height,
                        font=font)
        style.configure(pipe_deviated_style, background="blue", foreground="yellow", width=button_width,
                        height=button_height,
                        font=font)
        style.configure(fit_misplaced_style, background="cyan", foreground="yellow", width=button_width,
                        height=button_height,
                        font=font)
        style.configure(att_misplaced_style, background="magenta", foreground="yellow", width=button_width,
                        height=button_height,
                        font=font)
        style.configure(pipe_misplaced_style, background="blue", foreground="yellow", width=button_width,
                        height=button_height,
                        font=font)

        style.configure(treeview_style)
        style.layout(treeview_style, [(treeview_style + '.treearea', {'sticky': 'nswe'})])

        style.configure("TRadiobutton", anchor="W")



        if use_dark_theme:
            style.configure("TFrame", background = "grey12")
            style.configure("TButton", background="black", foreground="black")
            style.configure(free_style, background="black", foreground="black")
            style.configure(transition_style, background="tan4", foreground="black")
            style.configure("Treeview", foreground = "white", background = "grey12")
            style.configure("TLabelframe", foreground="white", background="grey12")
            style.configure("TLabelframe.Label", foreground="white", background="grey12")
            style.configure("TRadiobutton", background="grey12", foreground = "white")





        # Solution Display

        layout_frame = ttk.Frame(root)
        layout_frame.pack(anchor=tk.W)

        solution_button_grid_frame = ttk.LabelFrame(layout_frame, text="Solution Grid:")
        solution_button_grid_frame.grid(row=0, column=0)

        solution_button_grid, _, _ = get_button_grid(state_grid=state_grid,
                                                     button_grid_frame=solution_button_grid_frame,
                                                     start=start, goal=goal,
                                                     absolute_trail=self.process_planner.optimal_solution.absolute_trail,
                                                     process_planner=None,
                                                     part_select_option=None, tree=None, part_stock_tree=None,
                                                     solution_button_grid=None)

        # Current Build Layout Display

        construction_button_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=ttk.Button)

        construction_button_grid_frame = ttk.LabelFrame(layout_frame, text="Current construction grid:")
        construction_button_grid_frame.grid(row=1, column=0)

        part_stock_frame = ttk.LabelFrame(layout_frame, text="Current Parts:")
        part_stock_frame.grid(row=0, column=1)

        part_stock_tree = ttk.Treeview(part_stock_frame, columns=("part_id", "picked", "stock"), show="headings",
                                       style=treeview_style)
        part_stock_tree.heading("part_id", text="Part ID")
        part_stock_tree.heading("picked", text="Picked")
        part_stock_tree.heading("stock", text="Stock")
        part_stock_tree.column("part_id", width=50)
        part_stock_tree.column("picked", width=50)
        part_stock_tree.column("stock", width=50)
        part_stock_tree.grid(row=0, column=0)

        scrollbar = ttk.Scrollbar(part_stock_frame, orient=tk.VERTICAL, command=part_stock_tree.yview)
        part_stock_tree.configure(yscroll=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky='ns')

        tool_frame = ttk.LabelFrame(layout_frame, text="Process tools:")
        tool_frame.grid(row=1, column=1)

        part_put_frame = ttk.LabelFrame(tool_frame, text="Placement:")
        part_put_frame.grid(row=0, column=0)

        att_option_radiobutton = ttk.Radiobutton(part_put_frame, text="Attachment", variable=part_select_option,
                                                 value=3)
        att_option_radiobutton.pack(anchor=tk.W)

        pipe_option_radiobutton = ttk.Radiobutton(part_put_frame, text="Pipe", variable=part_select_option, value=2)
        pipe_option_radiobutton.pack(anchor=tk.W)

        fit_option_radiobutton = ttk.Radiobutton(part_put_frame, text="Fitting", variable=part_select_option,
                                                 value=1)
        fit_option_radiobutton.pack(anchor=tk.W)

        part_pick_frame = ttk.LabelFrame(tool_frame, text="Pick part:")
        part_pick_frame.grid(row=0, column=1)

        straight_pipe_ids = [k for k in path_problem.part_stock.keys()]

        process_message_frame = ttk.LabelFrame(layout_frame, text="Process Planner Message Output:")
        process_message_frame.grid(row=1, column=2)

        process_message_tree = ttk.Treeview(process_message_frame, show="tree", columns=("message",), selectmode="none")
        # process_message_tree.heading("message", text="Message")
        process_message_tree.column("#0", width=500)
        process_message_tree.column("message", width=0, anchor=tk.W)
        process_message_tree.grid(row=0, column=0)

        scrollbar = ttk.Scrollbar(process_message_frame, orient=tk.VERTICAL, command=process_message_tree.yview)
        process_message_tree.configure(style=treeview_style, yscroll=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky='ns')

        tool_tip_text_grid = []

        for part_id in straight_pipe_ids:
            pick_part_button = ttk.Button(part_pick_frame, text="Pick ID " + str(part_id),
                                          command=lambda t=part_id: send_new_pick_event(t,
                                                                                        process_planner=self.process_planner,
                                                                                        tree=process_message_tree,
                                                                                        part_stock_tree=part_stock_tree))
            pick_part_button.pack(anchor=tk.W)
            part_stock_tree.insert("", tk.END,
                                   values=(
                                       part_id,
                                       self.process_planner.tentative_process_state.picked_parts.count(part_id),
                                       self.process_planner.tentative_process_state.part_stock[part_id]))

        # todo: Construction Build Information Frame with picked_parts, current stock, deviation, solution scores etc.

        construction_definite_trail = get_absolute_trail_from_building_instructions(
            self.process_planner.last_process_state.building_instructions)

        construction_button_grid, style_grid, tool_tip_text_grid = get_button_grid(
            state_grid=self.process_planner.last_process_state.state_grid, start=start,
            goal=goal, button_grid_frame=construction_button_grid_frame,
            absolute_trail=construction_definite_trail,
            process_planner=self.process_planner,
            part_select_option=part_select_option,
            tree=process_message_tree, part_stock_tree=part_stock_tree, solution_button_grid=solution_button_grid)

        return_to_previous_state = ttk.Button(tool_frame, text="Undo action",
                                              command=lambda: undo_action(process_planner=self.process_planner,
                                                                          part_stock_tree=part_stock_tree,
                                                                          button_grid=construction_button_grid,
                                                                          style_grid=style_grid,
                                                                          tool_tip_text_grid=tool_tip_text_grid,
                                                                          process_message_tree=process_message_tree))
        return_to_previous_state.grid(row=2, column=0)

        # setting title
        root.title("Pipe Lab 2.0")
        # setting window size

        root.mainloop()
