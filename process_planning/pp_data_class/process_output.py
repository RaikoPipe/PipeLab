from dataclasses import dataclass
from typing import Union

from process_planning.pp_data_class.pick_event_info import PickEventInfo
from process_planning.pp_data_class.placement_event_info import PlacementEventInfo
from process_planning.process_state import ProcessState
from type_dictionary.type_aliases import Action, Pos


@dataclass
class ProcessOutput:
    """Output class used by process planner."""
    process_state: ProcessState
    """Modified process state after a motion event.
    
    *Type*: :class:`~process_state.ProcessState`"""
    current_event_info: Union[PlacementEventInfo, PickEventInfo]
    """Evaluated information on the current event. Can be a placement or pick event depending on the motion event input.
    
    *Type*: :obj:`Union` [:class:`~placement_event_info.PlacementEventInfo`, :class:`~pick_event_info.PickEventInfo`]"""
    messages: tuple[str]
    """A :obj:`tuple` containing :obj:`str` messages.
    These messages confirm a motion event or an error. For immediate comprehension.
    
    *Type*: :obj:`tuple` [:obj:`str`]
    """
    picking_robot_commands: tuple
    """See :meth:`~process_planner.ProcessPlanner.determine_picking_robot_commands`
    
    *Type*: :obj:`tuple`
    """
    fastening_robot_commands: tuple
    """See :meth:`~process_planner.ProcessPlanner.determine_fastening_robot_commands`
    
    *Type*: :obj:`tuple`
    """
    next_recommended_action: Action
    """
    Next recommended worker action (Place part with ID x at pos (x,x))
    See parameter :paramref:`~process_planner.ProcessPlanner.next_recommended_action`
    
    *Type*: :obj:`~type_aliases.Action`"""

    valid_placement_positions: set[Pos]
    """
    :obj:`set` containing valid placement :obj:`~type_aliases.Pos` for the part ID that was picked.
    
    *Type*: :obj:`set` [:obj:`~type_aliases.Pos`]"""
