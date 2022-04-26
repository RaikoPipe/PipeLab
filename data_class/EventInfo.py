from dataclasses import dataclass
from typing import Optional

from type_dictionary.common_types import *
from datetime import datetime



# todo: documentation

@dataclass
class EventInfo:
    """Dataclass containing information about a placement/removal/error event."""
    event_pos: Pos
    event_code: int
    removal: bool
    layout: Optional[Trail]
    part_id: Optional[int]
    deviated: bool
    detour_event: dict
    misplaced: bool
    unnecessary: bool
    obstructed_obstacle: bool
    obstructed_part: bool
    completed_layouts: set

    part_not_picked: bool

    error: bool

    time_registered : datetime

    layout_changed : bool
