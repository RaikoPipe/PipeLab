from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional, Set
from path_finding.common_types import *
from data_class.Solution import Solution


# todo: documentation

@dataclass
class ConstructionState:
    """Dataclass that contains information about the construction state of a layout."""

    event_code: int
    part_id: Optional[int]  # placed part id
    deviated: bool
    misplaced: bool
    unnecessary: bool
