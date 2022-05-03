from __future__ import annotations

from dataclasses import dataclass, field

from type_dictionary.common_types import StateGrid


@dataclass
class Predecessor:
    """Dataclass that contains information about the predecessor of a node"""
    pos: tuple  # Coordinates of the predecessor
    part_to_successor: int  # part used at predecessor_node to get to successor
    part_to_predecessor: int  # part used at predecessor_node to get to successor
    direction: tuple  # direction the predecessor is facing
    state_grid: StateGrid = None  # state of state grid when predecessor was created
    part_stock: dict = field(default_factory=dict)  # state of the part_stock when predecessor was created
    path: tuple = field(default_factory=tuple)  # path from start until predecessor
