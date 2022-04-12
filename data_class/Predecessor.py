from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class Predecessor:
    """Data class that contains information about the predecessor of a node"""
    pos: tuple  # Coordinates of the predecessor
    part_used: int  # part used at predecessor_node to get to successor
    direction: tuple  # direction the predecessor is facing
    state_grid: np.ndarray = None  # state of state grid when predecessor was created
    part_stock: dict = field(default_factory=dict)  # state of the part_stock when predecessor was created
    path: tuple = field(default_factory=tuple)  # path from start until predecessor
