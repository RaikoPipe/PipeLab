from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Optional
import numpy as np

from FunctionTestGUI.gui_config import treeview_style, set_visualization_style
from FunctionTestGUI.gui_util import undo_action, send_new_pick_event, get_button_grid
from PathFinding.pf_data_class.PathProblem import PathProblem
from ProcessPlanning.ProcessPlanner import ProcessPlanner
from ProcessPlanning.ProcessState import ProcessState
from ProcessPlanning.util.pp_util import get_absolute_trail_from_building_instructions

button_dict = {}

previous_detour_trails = set()

class function_test_app:
    """GUI for debugging and visualisation of the process planner. Initializes and uses an instance of a process planner."""
    def __init__(self, path_problem: PathProblem, initial_state: Optional[ProcessState]):
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

        set_visualization_style(button_height, button_width, font, style)



        # Solution Display

        layout_frame = ttk.Frame(root)
        layout_frame.pack(anchor=tk.W)

        solution_button_grid_frame = ttk.LabelFrame(layout_frame, text="Solution Grid:")
        solution_button_grid_frame.grid(row=0, column=0)

        solution_button_grid, _, _ = get_button_grid(state_grid=self.process_planner.last_process_state.aimed_solution.state_grid,
                                                     button_grid_frame=solution_button_grid_frame,
                                                     start=start, goal=goal,
                                                     absolute_trail=self.process_planner.optimal_solution.absolute_trail,
                                                     process_planner=None,
                                                     part_select_option=None, tree=None, part_stock_tree=None,
                                                     solution_button_grid=None)

        # Current Build Layout Display

        construction_button_grid = np.zeros((path_problem.state_grid.shape[0], path_problem.state_grid.shape[1]), dtype=ttk.Button)

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
                                       self.process_planner.last_process_state.picked_parts.count(part_id),
                                       self.process_planner.last_process_state.part_stock[part_id]))

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
                                                                          process_message_tree=process_message_tree,
                                                                          solution_button_grid=solution_button_grid,
                                                                          ))
        return_to_previous_state.grid(row=2, column=0)

        # setting title
        root.title("Pipe Lab 2.0")
        # setting window size

        root.mainloop()
