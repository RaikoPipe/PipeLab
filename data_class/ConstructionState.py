from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional, Set
from path_finding.common_types import *
from data_class.Solution import Solution

# todo: documentation

@dataclass
class ConstructionState:
    """Contains information about the construction state of a layout.
    The following conditions need to be met to complete a layout:
    - at least 1 pos inside attachment_pos
    - at least 2 items inside fitting_pos
    - at least 1 item inside pipe_pos"""

    event_code: int
    part_id: Optional[int] # placed part id
    deviated: bool
    misplaced: bool
    unnecessary: bool