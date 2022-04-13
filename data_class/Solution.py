from dataclasses import dataclass, field
from data_class import PathProblem
from typing import Optional
from path_finding.common_types import *


@dataclass
class Solution:
    """Dataclass that contains information about the solution to a path problem. Contains a copy of the path problem."""

    rendering_dict: dict[Pos:int] # similar to definite_path, but in a dict for easy access to part positions
    total_definite_trail: dict[Pos:int]  # contains a trail where each position refers to an ID it is occupied by
    layouts: list[Trail] # contains
    state_grid: any  # state grid with inserted solution layout
    score: Optional[
        float]  # how good the solution is -> lower is better; Score can be None, if solution not provided by algorithm for example
    algorithm: Optional[str]  # what algorithm has been used to search; Can be None, if provided by other means
    # problem_solved: bool # if the solution has solved the path problem (last entry in path == goal)
    path_problem: PathProblem  # The path problem it refers to
    part_stock: dict[int:int]

    fc_set: set[UndirectedConnection] = field(default_factory=set)
    # todo: replace code using this with rendering_dict
    definite_path: DefinitePath= field(default_factory=list)  # solution path: Positions with parts used to get to position
