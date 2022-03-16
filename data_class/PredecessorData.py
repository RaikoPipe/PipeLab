from dataclasses import dataclass, field
from typing import Optional

@dataclass
class PredecessorData:
    """Data class that contains information about the predecessor of a node"""
    pos : Optional[tuple]
    part_used: Optional[int] # part used at predecessor_node to get to successor
    direction: Optional[tuple]
    state_grid: any
    part_stock:dict
    path: list = field(default_factory=list)
