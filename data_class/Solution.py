from dataclasses import dataclass, field
from typing import Optional

from data_class import PathProblem
from types.type_dictionary import *


@dataclass
class Solution:
    """Dataclass that contains information about the solution to a path problem. Contains a copy of the path problem."""

    # todo: refactor rendering dict according to type
    rendering_dict: dict[int:tuple[Pos, int, Pos]]  # Dict containing information on rendering of objects
    absolute_trail: dict[Pos:int]  # Dict containing all trail positions pointing to the part they are occupied by
    ordered_trails: list[Trail]  # List with all trails in order from start to goal
    state_grid: any  # state grid with inserted solution layout
    score: Optional[
        float]  # measure of solution quality -> lower is better
    algorithm: Optional[str]  # what algorithm has been used to search
    path_problem: PathProblem  # The path problem the solution solves
    part_stock: dict[int:int]

    # todo: replace code using this with rendering_dict
    definite_path: DefinitePath = field(
        default_factory=list)  # solution path: Positions with parts used to get to position
