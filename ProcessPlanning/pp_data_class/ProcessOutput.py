from typing import Union
from dataclasses import dataclass

from ProcessPlanning.ProcessState import ProcessState
from ProcessPlanning.pp_data_class.PickEventInfo import PickEventInfo
from ProcessPlanning.pp_data_class.PlacementEventInfo import PlacementEventInfo


@dataclass
class ProcessOutput:
    """Output class used by process planner."""
    process_state: ProcessState
    """See :class:`ProcessState`"""
    current_event_info: Union[PlacementEventInfo, PickEventInfo]
    """Event Info of the current event."""
    messages: tuple[str]
    """A :obj:`tuple` containing :obj:`str` messages and a modified state with the new solution applied (:obj:`tuple` [:obj:`str`, :class:`ProcessState`])."""
    picking_robot_commands: tuple
    """See :meth:`~ProcessPlanner.ProcessPlanner.determine_picking_robot_commands`"""
    fastening_robot_commands: tuple
    """See :meth:`~ProcessPlanner.ProcessPlanner.determine_fastening_robot_commands`"""
    next_recommended_action: tuple
    """See parameter :paramref:`~ProcessPlanner.ProcessPlanner.next_recommended_action`"""
    valid_placement_positions: set
    """Not implemented"""
