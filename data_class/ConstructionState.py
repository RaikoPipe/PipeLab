from dataclasses import dataclass, field
from data_class import PathProblem, LayoutState
from typing import Optional, Set
from path_finding.common_types import *
from data_class.Solution import Solution
import numpy as np


#todo: documentation

@dataclass
class State:
    """Data class that contains information about a process state"""

    state_grid : np.ndarray
    """state grid with currently occupied spots"""

    part_stock: dict
    """parts left in stock"""

    aimed_solution: Solution

    construction_layouts: dict[Trail:LayoutState] = field(default_factory=dict) # Assigns Positions of a layout to a layout state -> current build state

    picked_parts: list[int]  = field(default_factory=list) # counts parts that have been picked. Counter reduces when parts are confirmed in a construction event

    # variables used for deviation detection/handling
    deviated: bool = False
    fc_set: set[DirectedConnection] = field(default_factory=set) # set of fitting connections that have been confirmed by a construction event
    removed_fc_set: set[UndirectedConnection] = field(default_factory=set) # set remembers fitting connections that have previously
    # existed before deconstruction until two removal events have occurred.
    deviated_motion_pos_fitting : set = field(default_factory=set) # list of placed fitting motions at pos
    deviated_motion_pos_attachment : set = field(default_factory=set) # list of placed attachment motions at pos
    deviated_motion_pos_pipe : dict = field(default_factory=dict) # list of placed pipe motions at pos


    unnecessary_parts : dict[Pos: int] = field(default_factory=dict) # dictionary that contains information about unnecessary parts according to the aimed solution
    misplaced_parts : dict[Pos: int]= field(default_factory=dict)
    #error_dict : dict[Pos: int] = field(default_factory=dict)


    latest_layout : Trail = field(default_factory=list)# a layout of the solution that is currently being build

    # latest commands
    pick_robot_commands : list[int] = field(default_factory=list)
    screw_robot_commands : list[int] = field(default_factory=list)
    obsolete_attachment_pos : set[Pos] = field(default_factory=set)

    action_info : dict = None








