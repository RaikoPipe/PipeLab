from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from PathFinding.pf_data_class import PathProblem
from type_dictionary.common_types import *


@dataclass
class Solution:
    """Dataclass that contains information about the solution to a path problem."""

    rendering_dict: dict[int, tuple[Pos, int]]  # Dict containing information on VpythonRendering of objects
    absolute_trail: dict[Pos, int]  #: Dict containing all trail positions pointing to the part they are occupied by
    ordered_trails: list[Trail]  #: List with all trails in order from start to goal
    state_grid: StateGrid  #: state grid with inserted solution layout
    score: Optional[
        float]  #: measure of solution quality. The lower the score the better the solution.
    algorithm: Optional[str]  #: what algorithm has been used to search
    path_problem: PathProblem  #: :class:`~PathProblem`
    part_stock: dict[int, int]  #: The amount of parts still available after applying the solution

    absolute_path: list = field(
        default_factory=list)  #: Positions with parts used to get to position
