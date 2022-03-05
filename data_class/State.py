from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional, Set
from path_finding.common_types import *

@dataclass
class State:
    """Data class that contains information about a process state"""
    state_grid : any # state grid with currently occupied spots
    part_stock: dict  # parts left
    definite_path: Optional[DefinitePath]  # only used when deviation detected to create partial solutions
    layouts: Layouts # contains instances of definite_path; represents current build state
    construction_trails: dict[Trail:int] # contains trails of occupied pipe pos after a construction_event.
    # First and last entry of trails are used fittings
    # 2: pipe is attached, 1: attachment is attached, 0: Nothing is attached (Trail is remembered, anyway)
    picked_parts: dict[int:int] # counts parts that have been picked. Counter reduces when parts are confirmed in a construction event
    connection_count: dict[Pos:int] # used for checking how many times a fitting has already been connected

    fc_set: set[UndirectedConnection] # set of fitting connections that have been confirmed by a construction event
    removed_fc_set: set[UndirectedConnection] # set remembers fitting connections that have previously
    # existed before deconstruction until two removal events have occurred.

    motion_dict : dict
    motion_fitting_pos : set # list of placed fitting motions at pos
    motion_attachment_pos : set # list of placed attachment motions at pos
    motion_pipe_pos : set # list of placed pipe motions at pos



    deviated: bool
