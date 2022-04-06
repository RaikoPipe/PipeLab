from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional
from path_finding.common_types import *


@dataclass
class Solution:
    """Data class that contains information about a path problem solution. Contains a copy of the path problem"""
    definite_path: DefinitePath  # solution path: Positions with parts used to get to position
    total_definite_trail: dict[Pos:int]  # contains a trail where each position refers to an ID it is occupied by
    layouts: list[Trail] # contains
    state_grid: any  # state grid with inserted solution layout
    score: Optional[
        float]  # how good the solution is -> lower is better; Score can be None, if solution not provided by algorithm for example
    algorithm: Optional[str]  # what algorithm has been used to search; Can be None, if provided by other means
    # problem_solved: bool # if the solution has solved the path problem (last entry in path == goal)
    path_problem: PathProblem  # The path problem it refers to

    fc_set: set[UndirectedConnection]
