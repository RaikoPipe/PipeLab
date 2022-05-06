from __future__ import annotations

from dataclasses import dataclass, field

from type_dictionary.type_aliases import *


@dataclass(slots=True)
class Predecessor:
    """Dataclass that contains information about the predecessor of a node"""
    pos: Pos  #
    """Node position of the predecessor.
    
    Type: :obj:`type_aliases.Pos`
    """
    part_to_successor: int  #
    """Part used at predecessor_node to get to successor.
    
    Type: :obj:`int`"""
    part_to_predecessor: int  #
    """Part used at predecessor_node to get to successor.
    
    Type: :obj:`int`"""
    direction: Pos  #
    """Direction the predecessor is facing.
    
    Type: :obj:`type_aliases.Pos`"""
    state_grid: StateGrid = None  #
    """State grid snapshot when predecessor was created.
    
    Type: :obj:`~type_aliases.StateGrid`"""
    part_stock: PartStock = field(default_factory=dict)  #
    """Snapshot of the part stock when predecessor was created.
    
    Type: :obj:`type_aliases.PartStock`"""
    path: Path = field(default_factory=tuple)
    """Snapshot of path to predecessor
    
    Type: :obj:`type_aliases.Path`"""
