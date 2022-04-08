import tkinter as tk
from copy import deepcopy
from tkinter import ttk
import numpy as np
from win32api import GetSystemMetrics

from ProcessPlanner import ProcessPlanner
from data_class.LayoutState import LayoutState
from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
from data_class.AssemblyState import State
from grid import grid_functions
from path_finding.common_types import *
from typing import Optional
from datetime import datetime

from pp_utilities import get_total_definite_trail_from_construction_layouts

free_style = "FREE.TButton"
pipe_style = "PIPE.TButton"
obs_style = "OBSTACLE.TButton"
att_style = "ATTACHMENT.TButton"
fit_style = "CORNER.TButton"
start_style = "START.TButton"
goal_style = "GOAL.TButton"
fit_caution_style = "FITCAUTION.TButton"
att_caution_style = "ATTCAUTION.TButton"
pipe_caution_style = "PIPECAUTION.TButton"
fit_warn_style = "FITWARN.TButton"
att_warn_style = "ATTWARN.TButton"
pipe_warn_style = "PIPEWARN.TButton"

button_dict = {}

def update_button_grid(event_info, button_grid, process_planner, style_grid, initial_style_grid, previous_style_grids):
    current_layout: Trail = event_info.get("current_layout")
    layout_state: LayoutState = event_info.get("layout_state")

    previous_style_grids.insert(0, deepcopy(style_grid))

    if current_layout:
        for pos in current_layout:
            style_grid[pos] = free_style

        # set fittings
        for pos in layout_state.fit_set:
            if pos in layout_state.required_fit_positions:
                style_grid[pos] = fit_style

        # set attachment
        for pos in layout_state.att_set:
            style_grid[pos] = att_style

        # set pipe
        if layout_state.pipe_set:
            for pos in current_layout:
                if pos not in layout_state.required_fit_positions:
                    style_grid[pos] = pipe_style

        for (pos, event_code) in process_planner.tentative_state.unnecessary_parts.items():
            if event_code == 1:
                style_grid[pos] = fit_caution_style

            if event_code == 3:
                style_grid[pos] = att_caution_style

        for (pos, event_code) in process_planner.tentative_state.misplaced_parts.items():
            if event_code == 1:
                style_grid[pos] = fit_warn_style

            if event_code == 2:
                style_grid[pos] = pipe_warn_style

            if event_code == 3:
                style_grid[pos] = att_warn_style

        pos_list = []
        for pos in process_planner.tentative_state.deviated_motion_set_fitting:
            pos_list.append(pos)
            style_grid[pos] = fit_caution_style
        for pos in process_planner.tentative_state.deviated_motion_set_attachment:
            pos_list.append(pos)
            style_grid[pos] = att_caution_style
        for pos in process_planner.tentative_state.deviated_motion_set_pipe.keys():
            pos_list.append(pos)
            style_grid[pos] = pipe_caution_style


        for pos, style in np.ndenumerate(initial_style_grid):
            if pos not in pos_list and pos not in process_planner.tentative_state.unnecessary_parts.keys() and pos not in process_planner.tentative_state.misplaced_parts.keys():
                if style_grid[pos] in (fit_caution_style, pipe_caution_style, att_caution_style):
                    style_grid[pos] = initial_style_grid[pos]

        style_grid[process_planner.tentative_state.aimed_solution.path_problem.start_pos] = start_style
        style_grid[process_planner.tentative_state.aimed_solution.path_problem.goal_pos] = goal_style

        for pos, style in np.ndenumerate(style_grid):
            button_grid[pos].configure(style=style)


def undo_action(process_planner, button_grid, style_grid, part_stock_tree, previous_style_grids):
    process_planner.return_to_previous_state()

    style_grid = previous_style_grids.pop(0)
    for pos, style in np.ndenumerate(style_grid):
        button_grid[pos].configure(style=style)

    part_id = 0
    for item in part_stock_tree.get_children():
        part_stock_tree.item(item, values=(part_id, process_planner.tentative_state.picked_parts.count(part_id),
                                           process_planner.tentative_state.part_stock[part_id]))
        part_id += 1

message_count = 0
def send_new_placement_event(pos, event_code, process_planner: ProcessPlanner, button_grid, tree, style_grid,
                             initial_style_grid, part_stock_tree, previous_style_grids):
    tentative_state = process_planner.main_func((pos, event_code), check_for_deviations=True, ignore_errors=False)
    event_info = tentative_state.event_info
    update_button_grid(event_info, button_grid, process_planner, style_grid, initial_style_grid, previous_style_grids)
    global message_count
    if event_info.get("message"):
        tree.insert("", index=tk.END, tag=message_count, values=(event_info["message"].replace("ProcessPlanner: ", ""),))
        message_count += 1
    if event_info.get("special_message"):
        tree.insert("", index=tk.END, tag=message_count, values=(event_info["special_message"].replace("ProcessPlanner: ", ""),))
        tree.tag_configure(tagname=message_count, background="yellow")
        message_count += 1
    if event_info.get("error_message"):
        tree.insert("", index=tk.END, tag=message_count, values=(event_info["error_message"].replace("ProcessPlanner: ", ""),))
        tree.tag_configure(tagname=message_count, background="red")
        message_count += 1

    part_id = event_info.get("pipe_id")

    if part_id is not None:
        item = part_stock_tree.get_children()[part_id]
        part_stock_tree.item(item, values=(part_id, process_planner.tentative_state.picked_parts.count(part_id),
                                           process_planner.tentative_state.part_stock[part_id]))
    tree.yview_moveto(1)


def send_new_pick_event(part_id, process_planner: ProcessPlanner, tree, part_stock_tree, style_grid,
                        previous_style_grids):
    message = process_planner.new_pick_event(part_id)
    tree.insert("", index=tk.END, values=(message,))
    item = part_stock_tree.get_children()[part_id]
    part_stock_tree.item(item, values=(part_id, process_planner.tentative_state.picked_parts.count(part_id),
                                       process_planner.tentative_state.part_stock[part_id]))
    previous_style_grids.insert(0, deepcopy(style_grid))
    tree.yview_moveto(1)


def get_button_grid(state_grid: np.ndarray, total_definite_trail, start, goal, button_grid_frame,
                    process_planner: Optional[ProcessPlanner],
                    part_select_option: Optional[tk.IntVar], tree, part_stock_tree):
    previous_style_grids = []
    # Grid Button
    button_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=ttk.Button)
    style_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=np.dtype("U100"))
    for pos, state in np.ndenumerate(state_grid):
        style = ""
        if state == 0:
            style = free_style
            style_grid[pos] = free_style
        elif state == 1:
            style = obs_style
            style_grid[pos] = obs_style
        elif state == 2:
            # what part?
            if total_definite_trail[pos] == 0:
                style = fit_style
                style_grid[pos] = fit_style
            else:
                style = pipe_style
                style_grid[pos] = pipe_style

        if pos == start:
            style = start_style
            style_grid[pos] = start_style
        elif pos == goal:
            style_grid[pos] = start_style
            style = goal_style

        initial_style_grid = deepcopy(style_grid)

        if process_planner:
            button = ttk.Button(button_grid_frame, text=str(pos), style=style)
            command = lambda t=pos: send_new_placement_event(t, part_select_option.get(), process_planner, button_grid,
                                                             tree, style_grid, initial_style_grid,
                                                             previous_style_grids=previous_style_grids,
                                                             part_stock_tree=part_stock_tree)
            button.config(command=command)
        else:
            button = ttk.Button(button_grid_frame, text=str(pos), style=style)
        button.grid(row=pos[0], column=pos[1], ipady=6, ipadx=3)
        button_grid[pos] = button

    previous_style_grids.insert(0, deepcopy(style_grid))

    return button_grid, style_grid, previous_style_grids


class function_test_app:
    def __init__(self, state_grid: np.ndarray, path_problem: PathProblem, initial_state: Optional[State]):
        root = tk.Tk()

        self.process_planner = ProcessPlanner(initial_path_problem=path_problem, initial_state=initial_state)


        start = path_problem.start_pos
        goal = path_problem.goal_pos

        part_select_option = tk.IntVar(value=1)
        pos_hovered = start

        style = ttk.Style()
        button_width = 5
        button_height = 5
        font= ('Tahoma', 7)
        style.configure(free_style, background="white", width=button_width, height=button_height, font=font)
        style.configure(pipe_style, background="blue", width=button_width, height=button_height, font=font)
        style.configure(obs_style, background="orange", width=button_width, height=button_height, font=font)
        style.configure(fit_style, background="cyan", width=button_width, height=button_height, font=font)
        style.configure(start_style, background="green", width=button_width, height=button_height, font=font)
        style.configure(goal_style, background="red", width=button_width, height=button_height, font=font)
        style.configure(att_style, background="magenta", width=button_width, height=button_height, font=font)
        style.configure(fit_caution_style, background="cyan", foreground="yellow", width=button_width, height=button_height,
                        font=font)
        style.configure(att_caution_style, background="magenta", foreground="yellow", width=button_width, height=button_height,
                        font=font)
        style.configure(pipe_caution_style, background="blue", foreground="yellow", width=button_width, height=button_height,
                        font=font)
        style.configure(fit_warn_style, background="cyan", foreground="red", width=button_width, height=button_height, font=font)
        style.configure(att_warn_style, background="magenta", foreground="red", width=button_width, height=button_height,
                        font=font)
        style.configure(pipe_warn_style, background="blue", foreground="red", width=button_width, height=button_height,
                        font=font)

        style.configure("TRadiobutton", anchor="W")

        # Solution Display

        layout_frame = ttk.Frame(root)
        layout_frame.pack(anchor=tk.W)

        solution_button_grid_frame = ttk.LabelFrame(layout_frame, text="Solution Grid:")
        solution_button_grid_frame.grid(row=0, column=0)

        solution_button_grid, _, _ = get_button_grid(state_grid=state_grid,
                                                     button_grid_frame=solution_button_grid_frame,
                                                     start=start, goal=goal,
                                                     total_definite_trail=self.process_planner.optimal_solution.total_definite_trail,
                                                     process_planner=None,
                                                     part_select_option=None, tree=None, part_stock_tree=None)

        # Current Build Layout Display

        construction_button_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=ttk.Button)

        construction_button_grid_frame = ttk.LabelFrame(layout_frame, text="Current construction grid:")
        construction_button_grid_frame.grid(row=1, column=0)

        part_stock_frame = ttk.LabelFrame(layout_frame, text="Current Parts:")
        part_stock_frame.grid(row=0, column=1)

        part_stock_tree = ttk.Treeview(part_stock_frame, columns=("part_id", "picked", "stock"), show="headings")
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

        process_message_tree = ttk.Treeview(process_message_frame, columns="message", show="headings")
        process_message_tree.heading("message", text="Message")
        process_message_tree.column("message", width=500)
        process_message_tree.grid(row=0, column=0)

        scrollbar = ttk.Scrollbar(process_message_frame, orient=tk.VERTICAL, command=process_message_tree.yview)
        process_message_tree.configure(yscroll=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky='ns')

        previous_style_grids = []

        for part_id in straight_pipe_ids:
            return_to_previous_state = ttk.Button(part_pick_frame, text="Pick ID " + str(part_id),
                                                  command=lambda t=part_id: send_new_pick_event(t,
                                                                                                process_planner=self.process_planner,
                                                                                                tree=process_message_tree,
                                                                                                part_stock_tree=part_stock_tree,
                                                                                                style_grid=style_grid,
                                                                                                previous_style_grids=previous_style_grids))
            return_to_previous_state.pack(anchor=tk.W)
            part_stock_tree.insert("", tk.END,
                                   values=(part_id, self.process_planner.tentative_state.picked_parts.count(part_id),
                                           self.process_planner.tentative_state.part_stock[part_id]))

        # todo: Construction Build Information Frame with picked_parts, current stock, deviation, solution scores etc.

        construction_definite_trail = get_total_definite_trail_from_construction_layouts(
            self.process_planner.latest_state.construction_layouts)

        construction_button_grid, style_grid, previous_style_grids = get_button_grid(
            state_grid=self.process_planner.latest_state.state_grid, start=start,
            goal=goal, button_grid_frame=construction_button_grid_frame,
            total_definite_trail=construction_definite_trail,
            process_planner=self.process_planner,
            part_select_option=part_select_option,
            tree=process_message_tree, part_stock_tree=part_stock_tree)

        return_to_previous_state = ttk.Button(tool_frame, text="Undo action",
                                              command=lambda: undo_action(process_planner=self.process_planner,
                                                                          part_stock_tree=part_stock_tree,
                                                                          button_grid=construction_button_grid,
                                                                          style_grid=style_grid,
                                                                          previous_style_grids=previous_style_grids))
        return_to_previous_state.grid(row=2, column=0)

        # setting title
        root.title("Pipe Lab")
        # setting window size

        root.mainloop()
