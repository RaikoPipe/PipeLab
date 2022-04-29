from dataclasses import dataclass
from datetime import datetime
from typing import Optional

from ProcessPlanning.classes.ProcessState import ProcessState
from ProcessPlanning.classes.data_class.EventInfo import EventInfo
from type_dictionary.common_types import Pos

@dataclass
class ProcessOutput:
    """Output class used by process planner."""
    process_state: ProcessState
    event_info: EventInfo
    messages: tuple
    next_recommended_action: tuple
    highlight_positions: tuple
