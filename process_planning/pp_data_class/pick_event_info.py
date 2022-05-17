from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime


@dataclass
class PickEventInfo():
    """Dataclass containing information about a pick event."""

    event_code: int
    """The Type of motion event.

    *Type*: :obj:`int`"""

    part_id : int
    """Part ID of the part picked.

    *Type*: :obj:`int`"""

    stock_empty: set
    """Set containing part IDs with empty stock.

    *Type*: :obj:`set`"""

    time_registered: datetime
    """Datetime of when event was registered.

    *Type*: :obj:`datetime`"""

    error: bool
    """If an error occurred.
    Errors signify that an action occurred that should have been impossible in the context of the current process state."""

    part_not_available: bool
    """Error specifier. If a part was picked that was empty in stock

    *Type*: :obj:`bool`"""







