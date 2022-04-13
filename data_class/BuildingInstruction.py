from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional, Set
from path_finding.common_types import *
from data_class.Solution import Solution


# todo: documentation

@dataclass
class BuildingInstruction:
    """Contains instructions for a construction layout.
    :param pipe_id:
    Args:
        pipe_id:
        """

    pipe_id: int  # ID of pipe that needs to be
    required_fit_positions: DirectedConnection  # Position where fittings need to be placed
    possible_att_pipe_positions: Trail
    recommended_attachment_pos: Pos  # recommended placement position of attachment, determined by an algorithm
    completed: bool = False  # if the layout is completed
