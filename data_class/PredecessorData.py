from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PredecessorData:
    """Data class that contains information about the predecessor of a node"""
    pos: Optional[tuple]  # Coordinates of the predecessor
    part_used: Optional[int]  # part used at predecessor_node to get to successor
    direction: Optional[tuple]  # direction the predecessor is facing
    state_grid: any  # state of state grid when predecessor was created
    part_stock: dict  # state of the part_stock when predecessor was created
    path: list = field(default_factory=list)  # path from start until predecessor
