

from dataclasses import dataclass
from datetime import datetime
from typing import Optional

from type_dictionary.type_aliases import Pos


# todo: documentation

@dataclass
class ConstructionState:
    """Used to describe a motion event that occurred at a position.
     Used in :class:`ProcessState<process_state>` to remember Motion Events in a :obj:`~class_types.MotionDict`."""
    event_pos: Pos
    """Node position where motion event was detected.
    
    *Type*: :obj:`~type_aliases.Pos`"""

    event_code: int
    """The Type of motion event.

    *Type*: :obj:`int`"""
    part_id: Optional[int]  # placed part id
    """Part ID that was placed/removed.
    
    *Type*: :obj:`~type_aliases.Pos`"""

    deviated: bool
    """If action caused a process deviation.
    
    *Type*: :obj:`bool`"""
    misplaced: bool
    """Specifier if deviated: If part that was placed with action occupies a node where a different part was recommended.
    
    *Type*: :obj:`~type_aliases.Pos`"""
    unnecessary: bool
    """
    Specifier if deviated: If part that was placed is not necessary for the layout. For example: multiple attachments in a layout.
    
    *Type*: :obj:`bool`"""
    time_registered: datetime
    """Datetime of when event was registered.
    
    *Type*: :obj:`datetime`"""