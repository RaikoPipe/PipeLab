import tkinter as tk
from copy import deepcopy
from tkinter import ttk
import numpy as np
from win32api import GetSystemMetrics

from ProcessPlanner import ProcessPlanner
from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
from data_class.State import State
from grid import grid_functions
from path_finding.common_types import *
from typing import Optional

from pp_utilities import get_total_definite_trail_from_construction_layouts

free_style = "FREE.TButton"
pipe_style = "PIPE.TButton"
obs_style = "OBSTACLE.TButton"
corn_style = "CORNER.TButton"
start_style = "START.TButton"
goal_style = "GOAL.TButton"

button_dict = {}


def send_new_placement_event(pos, event_code, process_planner:ProcessPlanner):
    print(str(pos), str(event_code))
    process_planner.new_construction_check((pos, event_code))

def send_new_pick_event(part_id, process_planner:ProcessPlanner):
    process_planner.new_pick_event(part_id)

def get_button_grid(state_grid: np.ndarray, total_definite_trail, start, goal, button_grid_frame,
                    process_planner: Optional[ProcessPlanner],
                    part_select_option: Optional[tk.IntVar]):
    # Grid Button
    button_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=ttk.Button)
    for pos, state in np.ndenumerate(state_grid):
        style = ""
        if state == 0:
            style = free_style
        elif state == 1:
            style = obs_style
        elif state == 2:
            # what part?
            if total_definite_trail[pos] == 0:
                style = corn_style
            else:
                style = pipe_style

        if pos == start:
            style = start_style
        elif pos == goal:
            style = goal_style

        if process_planner:
            button = ttk.Button(button_grid_frame, text=str(pos), style=style)
            command = lambda t=pos: send_new_placement_event(t, part_select_option.get(), process_planner)
            button.config(command=command)
        else:
            button = ttk.Button(button_grid_frame, text=str(pos), style=style)
        button.grid(row=pos[0], column=pos[1])
        button_grid[pos] = button

    return button_grid


class function_test_app:
    def __init__(self, state_grid: np.ndarray, path_problem: PathProblem, initial_state: Optional[State]):
        root = tk.Tk()

        self.process_planner = ProcessPlanner(initial_path_problem=path_problem, initial_state=initial_state)

        start = path_problem.start_pos
        goal = path_problem.goal_pos

        part_select_option = tk.IntVar()
        pos_hovered = start

        style = ttk.Style()
        style.configure(free_style, background="white", width=7, height=7, font=('Helvetica', 8))
        style.configure(pipe_style, background="blue", width=7, height=7, font=('Helvetica', 8))
        style.configure(obs_style, background="orange", width=7, height=7, font=('Helvetica', 8))
        style.configure(corn_style, background="cyan", width=7, height=7, font=('Helvetica', 8))
        style.configure(start_style, background="green", width=7, height=7, font=('Helvetica', 8))
        style.configure(goal_style, background="red", width=7, height=7, font=('Helvetica', 8))

        style.configure("TRadiobutton", anchor = "W")

        # Solution Display

        layout_frame = ttk.Frame(root)
        layout_frame.pack(anchor=tk.W)

        grid_label = ttk.Label(layout_frame, text="Solution Grid:")
        grid_label.grid(row=0, column=0)

        solution_button_grid_frame = ttk.Frame(layout_frame)
        solution_button_grid_frame.grid(row=0, column=0)

        solution_button_grid = get_button_grid(state_grid=state_grid, button_grid_frame=solution_button_grid_frame,
                                               start=start, goal=goal,
                                               total_definite_trail=self.process_planner.optimal_solution.total_definite_trail,
                                               process_planner=None,
                                               part_select_option=None)

        # Current Build Layout Display



        construction_button_grid = np.zeros((state_grid.shape[0], state_grid.shape[1]), dtype=ttk.Button)

        construction_grid_label = ttk.Label(layout_frame, text="Current Construction Grid:")
        construction_grid_label.grid(row=1, column=0)

        construction_button_grid_frame = ttk.Frame(layout_frame)
        construction_button_grid_frame.grid(row=2, column=0)



        construction_definite_trail = get_total_definite_trail_from_construction_layouts(
            self.process_planner.latest_state.construction_layouts)

        construction_button_grid = get_button_grid(state_grid=self.process_planner.latest_state.state_grid, start=start,
                                                   goal=goal, button_grid_frame=construction_button_grid_frame,
                                                   total_definite_trail=construction_definite_trail,
                                                   process_planner=self.process_planner,
                                                   part_select_option=part_select_option)

        tool_frame = ttk.Frame(layout_frame)
        tool_frame.grid(row=2, column=1)

        part_put_frame = ttk.Frame(tool_frame)
        part_put_frame.grid(row = 0, column=0)

        radio_label = ttk.Label(part_put_frame, text = "Part Type:")
        radio_label.pack(anchor=tk.W)

        att_option_radiobutton = ttk.Radiobutton(part_put_frame, text="Attachment", variable=part_select_option,
                                                 value=-1)
        att_option_radiobutton.pack(anchor=tk.W)

        pipe_option_radiobutton = ttk.Radiobutton(part_put_frame, text="Pipe", variable=part_select_option, value=1)
        pipe_option_radiobutton.pack(anchor=tk.W)

        fit_option_radiobutton = ttk.Radiobutton(part_put_frame, text="Fitting", variable=part_select_option,
                                                 value=0)
        fit_option_radiobutton.pack(anchor=tk.W)

        part_select_frame = ttk.Frame(tool_frame)
        part_select_frame.grid(row = 0, column=1)

        radio_label = ttk.Label(part_put_frame, text = "Pick pipe ID:")
        radio_label.pack(anchor=tk.W)

        straight_pipe_ids = [k for k in path_problem.part_stock.keys() if k != 0]

        for part_id in straight_pipe_ids:
            part_pick_button = ttk.Button(part_put_frame, text= "Pick ID "+ str(part_id),
                                          command = lambda t=part_id: send_new_pick_event(t, process_planner=self.process_planner))
            part_pick_button.pack(anchor=tk.W)

        #todo: Construction Build Information Frame with picked_parts, current stock, deviation, solution scores etc.



        # #setting variables
        # CameraOption = tk.StringVar()
        # displayWallOption = tk.IntVar()
        # displayTopOption = tk.IntVar()
        # displayObstacleOption = tk.IntVar()
        # displayPipesOption = tk.IntVar()
        # infoOption = tk.IntVar()
        # topAndWallxSizeString = tk.StringVar()
        # topHeightString=tk.StringVar()
        # wallHeightString=tk.StringVar()
        # defaultOption = tk.IntVar()
        # backgroundComboBoxString = tk.StringVar()
        # resWidth = tk.StringVar()
        # resHeight = tk.StringVar()
        # coordinateInfoOption = tk.IntVar()
        # showTestingPathsOption = tk.IntVar()
        # showTestedPathsOption = tk.IntVar()
        # randomizeOption = tk.IntVar()
        # displayWallDotsOption= tk.IntVar()
        # displayTopDotsOption = tk.IntVar()
        # gCoption = tk.StringVar()
        # gPoption = tk.StringVar()
        # gMinOoption = tk.StringVar()
        #
        # #partstrings
        # pl1 = tk.StringVar()
        # pc1 = tk.StringVar()
        # pl2 = tk.StringVar()
        # pc2 = tk.StringVar()
        # pl3 = tk.StringVar()
        # pc3 = tk.StringVar()
        # pl4 = tk.StringVar()
        # pc4 = tk.StringVar()
        # pl5 = tk.StringVar()
        # pc5 = tk.StringVar()

        # setting title
        root.title("Pipe Lab")
        # setting window size
        width = GetSystemMetrics(0) -200
        height = GetSystemMetrics(1) -100
        screenwidth = root.winfo_screenwidth()
        screenheight = root.winfo_screenheight()
        alignstr = '%dx%d+%d+%d' % (width, height, (screenwidth - width) / 2, (screenheight - height) / 2)
        root.geometry(alignstr)

        # style = ttk.Style()
        # style.configure("TButton", font = ("Calibri", 10), justify = "center")
        # style.configure("TRadiobutton", font = ("Calibri", 10),justify="left", anchor = "w", width = 20)
        # style.configure("TCheckbutton", font = ("Calibri", 10), justify="left", anchor = "w", width = 20)
        # style.configure("Nowidth.TCheckbutton", font = ("Calibri", 10), justify="left", anchor = "w")
        # style.configure("TLabel", font = ("Calibri", 12), justify="left")
        # style.configure("Res.TLabel", font=("Calibri", 12), justify="left", width=30)
        # style.configure("TEntry", font = ("Calibri", 12), justify="center")
        # style.configure("Res.TEntry", font = ("Calibri", 12), justify="center", width =10)
        # style.configure("TFrame" , font = ("Calibri", 12), justify="center")
        #
        #
        # #create button
        # #CreateSceneButton=ttk.Button(root, text="Create Scene", command=createNewScene)
        # #CreateSceneButton.place(x=320,y=440,width=170,height=43)
        #
        #
        # #defaultParametersButton = ttk.Checkbutton(root, text="Set default Values", command=setDefaultObjectParameters, variable = defaultOption)
        # #defaultParametersButton.grid(row=14, column=0)
        #
        # #camera field
        # grid_label=ttk.Label(root, text = "Current Grid::")
        # grid_label.grid(row=0,column=1)
        #
        # TwoDFrontCamOption=ttk.Radiobutton(root, text ="2D Front View", variable = CameraOption, value= "2DFront")
        # TwoDFrontCamOption.grid(row=1,column=1)
        #
        # TwoDUpViewOption=ttk.Radiobutton(root, text ="2D Up View", variable = CameraOption, value= "2DUp")
        # TwoDUpViewOption.grid(row=2,column=1)
        #
        # #scene field
        # sceneLabel= ttk.Label(root, text = "Scene Parameters:")
        # sceneLabel.grid(row=0,column=2)
        #
        # backgroundCombobox = ttk.Combobox(root, values=["black", "white"], state="readonly")
        # backgroundCombobox.grid(row=1, column=2)
        #
        # resolutionLabelW= ttk.Label(root, text = "Width:")
        # resolutionLabelW.grid(row=2, column=2)
        #
        # resolutionEntryW = ttk.Entry(root, textvariable = resWidth)
        # resolutionEntryW.grid(row=3, column=2)
        #
        # resolutionLabelH = ttk.Label(root, text = "Height:")
        # resolutionLabelH.grid(row=4,column=2)
        #
        # resolutionEntryH = ttk.Entry(root, textvariable = resHeight)
        # resolutionEntryH.grid(row=5, column=2)
        #
        # #info field
        #
        # infoLabel= ttk.Label(root, text = "Info Parameters:")
        # infoLabel.grid(row=0,column=3)
        #
        # CoordinateInfoCheckButton=ttk.Checkbutton(root, text= "Show coordinate info", variable = coordinateInfoOption)
        # CoordinateInfoCheckButton.grid(row=1,column=3)
        #
        # #goal parameter field
        # #todo: set start and goal parameters based on dots
        #
        # #parameterOutputField
        # parameterOutputFrame = ttk.Frame(root)
        # parameterOutputFrame.place(x=320,y=250)
        #
        # parameterOutputLabel = ttk.Label(parameterOutputFrame, text="Parameter Output:", style = "Res.TLabel")
        # parameterOutputLabel.grid(row=0,column=0)
        #
        # searchTypeCombobox = ttk.Combobox(parameterOutputFrame, values=["multicriteria astar", "astar", "dijkstra", "best-first"], state="readonly")
        # searchTypeCombobox.grid(row=1, column=1)
        #
        # #LevelSelect
        #
        # levelSelectLabel= ttk.Label(root, text = "Level Select:")
        # levelSelectLabel.grid(row=0,column=4)
        #
        # levelCombobox = ttk.Combobox(root, values=["Level 1 (Easy)", "Level 2 (Medium)", "Level 3 (Hard)", "High Complexity", "Save 1", "Save 2", "Save 3", "Save 4", "Save 5"], state="readonly")
        # levelCombobox.grid(row=1, column=4)
        #
        #
        #
        # # debugField
        # debugOptionsLabel= ttk.Label(root, text = "Showcase options:")
        # debugOptionsLabel.grid(row=0,column=5)
        #
        # showTestingPathsCheckButton=ttk.Checkbutton(root, text= "Show path finding", variable = showTestingPathsOption, style = "Nowidth.TCheckbutton")
        # showTestingPathsCheckButton.grid(row=1,column=5)
        #
        # showTestedPathsCheckButton=ttk.Checkbutton(root, text= "Show tested positions", variable = showTestedPathsOption, style = "Nowidth.TCheckbutton")
        # showTestedPathsCheckButton.grid(row=2,column=5)
        #
        # #insert default values
        # backgroundCombobox.set("white")
        # searchTypeCombobox.set("multicriteria astar")
        # levelCombobox.set("Level 1 (Easy)")
        # TwoDFrontCamOption.invoke()

        root.mainloop()
