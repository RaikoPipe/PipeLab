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
    picked_parts: dict[int:int] # counts parts that have been picked. Counter reduces when parts are confirmed in a construction event
    connection_count: dict[Pos:int] # used for checking how many times a fitting has already been connected

    fc_set: set[UndirectedConnection]

    motion_dict : dict
    fitting_pos : list
    attachment_pos : list
    pipe_pos : list