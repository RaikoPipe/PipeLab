from dataclasses import dataclass
from datetime import datetime
from typing import Optional

from type_dictionary.common_types import Pos


# todo: documentation

@dataclass
class ConstructionState:
    """Dataclass that contains information about the construction state of a layout."""
    event_pos: Pos
    event_code: int
    part_id: Optional[int]  # placed part id
    deviated: bool
    misplaced: bool
    unnecessary: bool

    time_registered: datetime
