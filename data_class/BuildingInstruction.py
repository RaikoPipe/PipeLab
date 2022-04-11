from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional, Set
from path_finding.common_types import *
from data_class.Solution import Solution

# todo: documentation

@dataclass
class BuildingInstruction:
    """Contains information about the construction state of a layout.
    The following conditions need to be met to complete a layout:
    - at least 1 pos inside attachment_pos
    - at least 2 items inside fitting_pos
    - at least 1 item inside pipe_pos"""

    pipe_id: int # ID of pipe that needs to be
    required_fit_positions: DirectedConnection # Position where fittings need to be placed
    # todo: add possible att/pipe positions to each building instruction
    possible_att_pipe_positions: Trail
    recommended_attachment_pos: Pos # recommended placement position of attachment, determined by an algorithm
    completed : bool = False # if the layout is completed