from __future__ import annotations

from dataclasses import dataclass

from PathFinding.pf_data_class.Weights import Weights

from type_dictionary.common_types import *  #
from typing import Optional


# todo: documentation

@dataclass
class PathProblem:
    """Dataclass that contains detailed information about the path problem.

    Args:
        state_grid:
        start_pos:
        goal_pos:

        start_directions: directions that start is restricted to
        goal_directions: directions that goal is restricted to (i.e. last part that is linked to the goal)
        transition_points: Contains border markings, beyond which nodes lie in different plane

        starting_part: signifies if there is a starting part; None: No part; 0: Corner; 1: Pipe

        part_stock: amount of parts available for assembling a solution
        part_cost: dictionary that contains the replacement costs (or opportunity costs) of parts

        weights:

    """
    state_grid: np.ndarray
    start_pos: Pos
    goal_pos: Pos

    start_directions: set[Pos]
    goal_directions: set[Pos]
    transition_points: set[Pos]

    starting_part: Optional[int]

    part_stock: dict
    part_cost: dict

    # solving options
    weights: Optional[Weights]
    algorithm: Optional[str]
