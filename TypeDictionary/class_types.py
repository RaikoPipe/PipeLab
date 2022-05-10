from PathFinding.pf_data_class.Predecessor import Predecessor
from ProcessPlanning.pp_data_class.BuildingInstruction import BuildingInstruction
from ProcessPlanning.pp_data_class.ConstructionState import ConstructionState

from TypeDictionary.type_aliases import *

MotionDict: TypeAlias = dict[Union[Pos, Trail], ConstructionState]
"""See :class:`motion_dict in ProcessState<ProcessState>`


*Type*: (:obj:`dict` [:obj:`Union` [:obj:`~type_aliases.Pos`, :obj:`~type_aliases.Trail`], :class:`ConstructionState`])"""

BuildingInstructions: TypeAlias = dict[Trail, BuildingInstruction]
"""Dictionary containing trails pointing to their :class:`BuildingInstruction`.

*Type*: (:obj:`dict` [:obj:`~type_aliases.Trail`, :class:`BuildingInstruction`])"""

Predecessors: TypeAlias = dict[Union[Pos, tuple[Pos, int, Pos]], Predecessor]
"""Dictionary containing node positions pointing to their predecessors. 

*Type*: :obj:`dict` [:obj:`Union` [:obj:`~type_aliases.Pos`, :obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`int`, :obj:`~type_aliases.Pos`]], :class`Predecessor`]"""