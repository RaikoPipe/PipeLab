from dataclasses import dataclass

from type_dictionary.common_types import *

#todo: documentation

@dataclass
class BuildingInstruction:
    """Contains instructions for a construction layout.
        """

    pipe_id: int  # ID of pipe that needs to be
    required_fit_positions: DirectedConnection  # Position where fittings need to be placed
    possible_att_pipe_positions: Trail
    recommended_attachment_pos: Pos  # recommended placement position of attachment, determined by an algorithm
    layout_completed: bool = False  # if the layout is completed
