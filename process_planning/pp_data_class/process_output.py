from dataclasses import dataclass
from typing import Union

from process_planning.pp_data_class.pick_event_result import PickEventResult
from process_planning.pp_data_class.assembly_event_result import AssemblyEventResult
from process_planning.process_state import ProcessState
from type_dictionary.type_aliases import Action, Pos


@dataclass
class ProcessOutput:
    """Output dataclass for neatly outputting the return for
    :meth:`~process_planner.ProcessPlanner.handle_motion_event`."""

    process_state: ProcessState
    """Updated process state after a motion event.
    
    *Type*: :class:`ProcessState<process_state>`"""
    current_event_result: Union[AssemblyEventResult, PickEventResult, None]
    """
    Evaluated information on the current event. Can be either :class:`AssemblyEventResult <assembly_event_result>`
    or :class:`PickEventResult<pick_event_result>` depending on the evaluated motion event.
    
    *Type*: :obj:`Union` [:class:`AssemblyEventResult <assembly_event_result>`, :class:`PickEventResult<pick_event_result>`]"""
    messages: tuple[str]
    """A :obj:`tuple` containing :obj:`str` messages.
    These messages confirm a motion event or an error. For easily consumable feedback for the input motion event.
    
    *Type*: :obj:`tuple` [:obj:`str`]
    """
    picking_robot_commands: tuple
    """:obj:`tuple` containing robot command codes for the picking robot 

    See :ref:`Picking Robot Command Codes` for reference. 
    
    Determined in: :meth:`~process_planner.ProcessPlanner.determine_picking_robot_commands`
    
    *Type*: :obj:`tuple`
    """
    fastening_robot_commands: tuple
    """
    :obj:`tuple` containing robot command codes for the fastening robot 

    See :ref:`Fastening Robot Command Codes` for reference. 
    
    Determined in: :meth:`~process_planner.ProcessPlanner.determine_fastening_robot_commands`
    
    *Type*: :obj:`tuple`
    """
    next_recommended_action: Action
    """
    Next recommended worker action (Place part with ID x at pos (x,x))
    See parameter :paramref:`~process_planner.ProcessPlanner.next_recommended_action`.
    
    *Type*: :obj:`~type_aliases.Action`"""

    valid_assembly_positions: set[Pos]
    """
    :obj:`set` containing valid assembly :obj:`~type_aliases.Pos` for the part ID that was picked.
    
    *Type*: :obj:`set` [:obj:`~type_aliases.Pos`]"""
