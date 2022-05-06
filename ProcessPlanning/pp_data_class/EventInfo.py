from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Optional

from type_dictionary.type_aliases import *
from ProcessPlanning.pp_data_class.ConstructionState import ConstructionState
from type_dictionary.class_types import BuildingInstructions


@dataclass
class EventInfo(ConstructionState):
    """Dataclass containing information about a placement/removal/error event."""

    removal: bool
    """If a part was removed.
    
    Type: :obj:`bool`"""

    layout: Optional[Trail]
    """Layout where motion event happened.
    
    Type: :obj:`Optional` [:obj:`~type_aliases.Trail`]"""
    completed_layouts: set
    """The layouts that were completed by that action.
    
    Type: :obj:`~type_aliases.Pos`"""
    detour_event: Optional[BuildingInstructions]
    """
    
    Type: :ob:`Optional` [:obj:`~class_types.BuildingInstructions`]"""

    error: bool
    """If an error occurred.
    Errors signify that an action occurred that should have been impossible in the context of the current process state.
    
    Type: :obj:`bool`"""
    part_not_picked: bool
    """Error specifier. If a part was not picked.
    
    Type: :obj:`bool`"""
    obstructed_obstacle: bool
    """Error specifier. If an obstacle was obstructed by a placement.
    
    Type: :obj:`bool`"""
    obstructed_part: int
    """Error specifier. If another part was obstructed by a placement.
    
    Type: :obj:`int`"""

