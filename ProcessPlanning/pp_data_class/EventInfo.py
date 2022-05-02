from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Optional
from type_dictionary.common_types import *


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
    misplaced: bool
    unnecessary: bool

    completed_layouts: set

    detour_event: dict

    error: bool
    part_not_picked: bool
    obstructed_obstacle: bool
    obstructed_part: int

    time_registered: datetime
