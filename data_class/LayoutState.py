from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional, Set
from path_finding.common_types import *
from data_class.Solution import Solution

@dataclass
class LayoutState:
    """Contains information about the construction state of a layout.
    The following conditions need to be met to complete a layout:
    - at least 1 pos inside attachment_pos
    - at least 2 items inside fitting_pos
    - at least 1 item inside pipe_pos"""
    attachment_pos: set[Pos] # positions where attachments have been placed inside this layout
    pipe_pos:set[Pos] # positions where pipe attaching motions where detected inside this layout.
    fitting_pos: set[Pos]  # position where fittings have been placed inside this layout
    pipe_id: int
    correct_fitting_pos: DirectedConnection
    recommended_attachment_pos: Pos