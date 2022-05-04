from typing import Union

from common_types import *
from ProcessPlanning.pp_data_class.BuildingInstruction import BuildingInstruction
from ProcessPlanning.pp_data_class.ConstructionState import ConstructionState

MotionEvent = tuple[Union[Pos, int], int]
"""..."""

BuildingInstructions: TypeAlias = dict[Trail, BuildingInstruction]
"""..."""

Action: TypeAlias = tuple[Pos, int, int]
"""..."""

NodePair = tuple[Pos, Union[Pos, tuple]]
"""..."""

NodePairSet: TypeAlias = set[NodePair]
"""..."""

OrderedTrails: TypeAlias = list[Trail]
"""..."""

TrailList: TypeAlias = list[Trail]
"""..."""

DirectionDict: TypeAlias = dict[Pos, Pos]
"""..."""

PosSet = set[Pos]
"""..."""

MotionDict: TypeAlias = dict[tuple[Pos, int]:ConstructionState]
"""..."""
