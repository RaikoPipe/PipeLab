from typing import Union

from common_types import *
from ProcessPlanning.pp_data_class.BuildingInstruction import BuildingInstruction

MotionEvent = tuple[Union[Pos, int], int]

BuildingInstructions : TypeAlias = dict[Trail, BuildingInstruction]