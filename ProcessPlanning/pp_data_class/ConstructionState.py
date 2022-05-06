

from dataclasses import dataclass
from datetime import datetime
from typing import Optional

from type_dictionary.type_aliases import Pos


# todo: documentation

@dataclass
class ConstructionState:
    """Dataclass that contains information about the construction state of a layout."""
    event_pos: Pos
    """Node position where motion event was detected.
    
    Type: :obj:`~type_aliases.Pos`"""

    event_code: int
    """Type of motion event.

    Type: :obj:`int`"""
    part_id: Optional[int]  # placed part id
    """Part ID that was placed/removed.
    
    Type: :obj:`~type_aliases.Pos`"""

    deviated: bool
    """If action caused a process deviation.
    
    Type: :obj:`bool`"""
    misplaced: bool
    """If part that was placed with action occupies a node where a different part was recommended.
    
    Type: :obj:`~type_aliases.Pos`"""
    unnecessary: bool
    """If part that was placed is not necessary for the layout (for example: multiple attachments in a layout).
    
    Type: :obj:`bool`"""
    time_registered: datetime
    """Datetime of when event was registered.
    
    Type: :obj:`datetime`"""