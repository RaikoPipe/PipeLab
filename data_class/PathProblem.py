from dataclasses import dataclass, field
from typing import Optional
from data_class.Weights import Weights
from path_finding.common_types import Node
from path_finding.common_types import *#
import numpy as np

#todo: documentation

@dataclass
class PathProblem:
    """Data class that contains detailed information about the path problem."""
    state_grid: np.ndarray
    start_pos: Pos
    goal_pos: Pos

    start_directions: Positions # directions that start is restricted to
    goal_directions: Positions # directions that goal is restricted to (i.e. last part that is linked to the goal)

    starting_part: Optional[int] # signifies if there is a starting part; None: No part; 0: Corner; 1: Pipe

    part_stock: dict # amount of parts available for assembling a solution
    part_cost: dict # dictionary that contains the replacement costs (or opportunity costs) of parts

    #solving options
    weights: Optional[Weights]
    algorithm: Optional[str]
