from __future__ import annotations

from dataclasses import dataclass

from ProcessPlanning.ProcessState import ProcessState
from ProcessPlanning.pp_data_class.EventInfo import EventInfo


@dataclass
class ProcessOutput:
    """Output class used by process planner."""
    process_state: ProcessState
    event_info: EventInfo
    messages: tuple
    picking_robot_commands: tuple
    fastening_robot_commands: tuple
    next_recommended_action: tuple
    highlight_positions: tuple
