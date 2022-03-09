from dataclasses import dataclass
from data_class import PathProblem, LayoutState
from typing import Optional, Set
from path_finding.common_types import *
from data_class.Solution import Solution

@dataclass
class State:
    """Data class that contains information about a process state"""
    state_grid : any # state grid with currently occupied spots
    part_stock: dict  # parts left
    definite_path: Optional[DefinitePath]  # only used when deviation detected to create partial solutions
    construction_layouts: dict[Trail:LayoutState] # Assigns Positions of a layout to a layout state

    construction_parts: dict[Pos:int]  # signifies which pos are occupied by what part id
    picked_parts: dict[int:int] # counts parts that have been picked. Counter reduces when parts are confirmed in a construction event

    # variables used for deviation detection/handling
    connection_count: dict[Pos:int] # used for checking how many times a fitting has already been connected
    deviated: bool
    fc_set: set[UndirectedConnection] # set of fitting connections that have been confirmed by a construction event
    removed_fc_set: set[UndirectedConnection] # set remembers fitting connections that have previously
    # existed before deconstruction until two removal events have occurred.
    motion_fitting_pos : set # list of placed fitting motions at pos
    motion_attachment_pos : set # list of placed attachment motions at pos
    motion_pipe_pos : set # list of placed pipe motions at pos
    remove_parts : dict[Pos: int] # dictionary that contains information about unnecessary parts according to the aimed solution
    error_dict : dict[Pos: int]

    # latest commands
    pick_robot_commands = list[int]
    screw_robot_commands = list[int]

    aimed_solution = Solution
    latest_layout = Trail # a layout of the solution that is currently being build






