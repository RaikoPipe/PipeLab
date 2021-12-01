from dataclasses import dataclass, field
from typing import Optional

@dataclass
class PathProblem:
    """Data class that contains detailed information about the path problem."""
    state_grid: any
    start_node: tuple
    goal_node: tuple

    start_direction: tuple # direction that start is restricted to
    goal_direction: tuple # direction that goal is restricted to (i.e. last part that is linked to the goal)


    starting_part: Optional[int] # signifies if there is a starting part; None: No part; 0: Corner; 1: Pipe

    goal_is_transition: bool # direction of the transition point; Is None if goal is not a transition
    part_stock: dict #{point_length: amount}
    part_cost: dict # dictionary that contains the replacement costs (or opportunity costs) of parts