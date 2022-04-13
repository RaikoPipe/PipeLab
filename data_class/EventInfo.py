from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional, Set
from path_finding.common_types import *
from data_class.Solution import Solution

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
    rerouting_event: dict
    misplaced: bool
    unnecessary: bool
    obstructed_obstacle: bool
    obstructed_part: bool
    completed: bool

    part_not_picked: bool

    error: bool

