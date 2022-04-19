from dataclasses import dataclass
from typing import Optional

from types.type_dictionary import *


# todo: documentation

@dataclass
class EventInfo:
    """Dataclass containing information about a placement/removal/error event."""
    event_pos: Pos
    event_code: int
    removal: bool
    current_layout: Optional[Trail]
    part_id: Optional[int]
    deviated: bool
    detour_event: dict
    misplaced: bool
    unnecessary: bool
    obstructed_obstacle: bool
    obstructed_part: bool
    completed: bool

    part_not_picked: bool

    error: bool
