from dataclasses import dataclass
from data_class import PathProblem
from typing import Optional

@dataclass
class Solution:
    """Data class that contains information about a solution and the problem it refers to."""
    path: list # the calculated path
    parts: list # what parts need to be used to build layout
    solution_grid : any # state grid with inserted solution layout
    score: Optional[float] # how good the solution is -> lower is better; Score can be None, if solution not provided by algorithm for example
    algorithm: Optional[str] # what algorithm has been used to search; Can be None, if provided by other means
    problem_solved: bool # if the solution has solved the path problem (last entry in path == goal)
    path_problem: PathProblem # The path problem it refers to