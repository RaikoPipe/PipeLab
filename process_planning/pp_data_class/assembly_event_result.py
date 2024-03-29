from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from process_planning.pp_data_class.construction_state import ConstructionState
from type_dictionary.class_types import BuildingInstructions
from type_dictionary.type_aliases import *


@dataclass
class AssemblyEventResult(ConstructionState):
    """Used to describe the result of an assembly/removal motion event after it was evaluated by :class:`ProcessState<process_state>`"""

    removal: bool
    """If a part was removed.
    
    *Type*: :obj:`bool`"""

    layout: Optional[Trail]
    """Layout where motion event happened.
    
    *Type*: :obj:`Optional` [:obj:`~type_aliases.Trail`]"""
    completed_layouts: set
    """The layouts that were completed by that action.
    
    *Type*: :obj:`~type_aliases.Pos`"""
    detour_event: Optional[BuildingInstructions]
    """Dictionary containing a trail and building instruction of the deviated layout.
    
    *Type*: :obj:`Optional` [:obj:`~class_types.BuildingInstructions`]"""

    error: bool
    """If an error occurred.
    Errors signify that an action occurred that should have been impossible in the context of the current process state.
    
    *Type*: :obj:`bool`"""
    part_not_picked: bool
    """Error specifier. If a part was not picked.
    
    *Type*: :obj:`bool`"""
    obstructed_obstacle: bool
    """Error specifier. If an obstacle was obstructed by an assembly.
    
    *Type*: :obj:`bool`"""
    obstructed_part: int
    """Error specifier. If another part was obstructed by an assembly.
    
    *Type*: :obj:`int`"""

