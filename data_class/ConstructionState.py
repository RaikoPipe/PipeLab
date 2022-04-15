from dataclasses import dataclass
from typing import Optional


# todo: documentation

@dataclass
class ConstructionState:
    """Dataclass that contains information about the construction state of a layout."""

    event_code: int
    part_id: Optional[int]  # placed part id
    deviated: bool
    misplaced: bool
    unnecessary: bool
