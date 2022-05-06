from __future__ import annotations

from dataclasses import dataclass

from type_dictionary.type_aliases import *


@dataclass(slots=True)
class BuildingInstruction:
    """Contains instructions for building a layout.
        """

    pipe_id: int
    """ID of pipe that needs to be placed in the layout.
    
    Type: :obj:`int`"""
    required_fit_positions: DirectedConnection
    """Position where fittings need to be placed.
    
    Type: :obj:`~type_aliases.DirectedConnection`"""
    possible_att_pipe_positions: Trail  #: TODO: doc
    """Trail containing node positions that a pipe of this layout would occupy.
     Used for checking if position of a motion event is valid for attachment or pipe placement.
    
    Type: :obj:`~type_aliases.Trail`"""
    recommended_attachment_pos: Pos
    """Recommended placement position of attachment, determined by an algorithm.
    
    Type: :obj:`~type_aliases.Pos`"""
    layout_completed: bool = False
    """If the layout is completed.
    
    Type: :obj:`bool`"""
