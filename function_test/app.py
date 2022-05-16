import ttkbootstrap as ttk

from function_test.app_config import treeview_style, configure_style, dark_theme, dark_theme_use, light_theme_use
from function_test.app_util import undo_action, send_new_pick_event, get_button_grid
from path_finding.pf_data_class.path_problem import PathProblem
from process_planning.process_planner import ProcessPlanner

button_dict = {}

previous_detour_trails = set()


class FunctionTestApp:
    """Provides a UI for the process planner, mainly for debugging purposes. Visualizes the process state.
    Initializes and uses an instance of a process planner.

    """

    def __init__(self, path_problem: PathProblem, process_planner: ProcessPlanner, controlled_from_outside=False):
        """

        Args:
            path_problem:
            process_planner:
            controlled_from_outside:
        """

        if dark_theme:
            theme = dark_theme_use
        else:
            theme = light_theme_use
        self.root = ttk.Window(themename=theme)

        # set icon
        self.root.iconbitmap('function_test/resources/PipeLab2.ico')

        # set title
        self.root.title("PipeLab2 Function Test")



        style = ttk.Style(theme)
        configure_style(style)

        self.process_planner: ProcessPlanner = process_planner

        start = path_problem.start_pos
        goal = path_problem.goal_pos

        part_select_option = ttk.IntVar(value=1)

        # make app widgets

        layout_frame = ttk.Frame(self.root)
        layout_frame.grid(row=0, column=0)

        self.solution_button_grid_frame = ttk.LabelFrame(layout_frame, text="Solution Grid:")
        self.solution_button_grid_frame.grid(row=0, column=0)

        # Button grid where solution is visualized
        self.solution_button_grid, _, _ = get_button_grid(
            state_grid=self.process_planner.last_process_state.aimed_solution.state_grid,
            button_grid_frame=self.solution_button_grid_frame,
            start=start, goal=goal,
            absolute_trail=self.process_planner.optimal_solution.node_trail,
            process_planner=None,
            part_select_option=None, tree=None, part_stock_tree=None,
            solution_button_grid=None)

        self.construction_button_grid_frame = ttk.LabelFrame(layout_frame, text="Current construction grid:")
        self.construction_button_grid_frame.grid(row=1, column=0)

        pp_frame = ttk.LabelFrame(self.root, text="Tool Window")
        pp_frame.grid(row=0, column=1, padx=10)

        tool_frame = ttk.LabelFrame(pp_frame, text="Process tools:")
        tool_frame.grid(row=0, column=0)

        display_frame = ttk.LabelFrame(pp_frame, text="Process Planner Message Output:")
        display_frame.grid(row=1, column=0, pady=10)

        part_stock_frame = ttk.LabelFrame(tool_frame, text="Current Parts:")
        part_stock_frame.grid(row=0, column=2, padx=5)

        # set a view for displaying part stock, picked parts
        self.part_stock_tree = ttk.Treeview(part_stock_frame, columns=("part_id", "picked", "stock"), show="headings",
                                            style=treeview_style)
        self.part_stock_tree.heading("part_id", text="Part ID")
        self.part_stock_tree.heading("picked", text="Picked")
        self.part_stock_tree.heading("stock", text="Stock")
        self.part_stock_tree.column("part_id", width=60)
        self.part_stock_tree.column("picked", width=60)
        self.part_stock_tree.column("stock", width=60)
        self.part_stock_tree.grid(row=0, column=0)

        # set a scrollbar for the part stock view
        scrollbar = ttk.Scrollbar(part_stock_frame, orient=ttk.VERTICAL, command=self.part_stock_tree.yview)
        self.part_stock_tree.configure(yscroll=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky='ns')

        part_put_frame = ttk.LabelFrame(tool_frame, text="Placement:")
        part_put_frame.grid(row=0, column=0, padx=5)

        att_option_radiobutton = ttk.Radiobutton(part_put_frame, text="Attachment", variable=part_select_option,
                                                 value=3)
        att_option_radiobutton.pack(anchor=ttk.W, pady=5)

        # place pipe choice radio button
        pipe_option_radiobutton = ttk.Radiobutton(part_put_frame, text="Pipe", variable=part_select_option, value=2)
        pipe_option_radiobutton.pack(anchor=ttk.W, pady=5)

        # place fitting choice radio button
        fit_option_radiobutton = ttk.Radiobutton(part_put_frame, text="Fitting", variable=part_select_option, value=1)
        fit_option_radiobutton.pack(anchor=ttk.W, pady=5)

        part_pick_frame = ttk.LabelFrame(tool_frame, text="Pick part:")
        part_pick_frame.grid(row=0, column=1)

        process_message_frame = ttk.LabelFrame(display_frame, text="")
        process_message_frame.grid(row=0, column=1)

        # treeview for displaying process planner messages
        self.process_message_tree = ttk.Treeview(process_message_frame, show="tree", columns=("message",),
                                                 selectmode="none")
        self.process_message_tree.column("#0", width=500)
        self.process_message_tree.column("message", width=0, anchor=ttk.W)
        self.process_message_tree.grid(row=0, column=0)

        # set a scrollbar for the process message view
        scrollbar = ttk.Scrollbar(process_message_frame, orient=ttk.VERTICAL, command=self.process_message_tree.yview)
        self.process_message_tree.configure(style=treeview_style, yscroll=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky='ns')

        part_ids = [k for k in path_problem.part_stock.keys()]
        # set a button for picking each part id
        for part_id in part_ids:
            pick_part_button = ttk.Button(part_pick_frame, text="Pick ID " + str(part_id),
                                          command=lambda t=part_id: send_new_pick_event(
                                              t,
                                              process_planner=self.process_planner,
                                              process_message_tree=self.process_message_tree,
                                              part_stock_tree=self.part_stock_tree), style="primary.Outline.TButton")
            pick_part_button.pack(anchor=ttk.W)
            self.part_stock_tree.insert("", ttk.END,
                                        values=(
                                            part_id,
                                            self.process_planner.last_process_state.picked_parts.count(part_id),
                                            self.process_planner.last_process_state.part_stock[part_id]))

        # Button grid where current process state is visualized
        self.construction_button_grid, self.style_grid, self.tool_tip_text_grid = get_button_grid(
            state_grid=self.process_planner.last_process_state.state_grid, start=start,
            goal=goal, button_grid_frame=self.construction_button_grid_frame,
            absolute_trail=self.process_planner.last_process_state.aimed_solution.node_trail,
            process_planner=self.process_planner,
            part_select_option=part_select_option,
            tree=self.process_message_tree, part_stock_tree=self.part_stock_tree,
            solution_button_grid=self.solution_button_grid)

        # set button to revert to a previous process state
        return_to_previous_state = ttk.Button(tool_frame, text="Undo action", style="danger.TButton",
                                              command=lambda: undo_action(process_planner=self.process_planner,
                                                                          part_stock_tree=self.part_stock_tree,
                                                                          button_grid=self.construction_button_grid,
                                                                          style_grid=self.style_grid,
                                                                          tool_tip_text_grid=self.tool_tip_text_grid,
                                                                          process_message_tree=self.process_message_tree,
                                                                          solution_button_grid=self.solution_button_grid,
                                                                          ))
        return_to_previous_state.grid(row=2, column=0)

        if controlled_from_outside:
            # update once

            self.root.update_idletasks()
            self.root.update()
        else:
            self.root.mainloop()
