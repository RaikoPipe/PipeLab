from path_finding.pf_data_class.predecessor import Predecessor
from process_planning.pp_data_class.building_instruction import BuildingInstruction
from process_planning.pp_data_class.construction_state import ConstructionState

from type_dictionary.type_aliases import *

MotionDict: TypeAlias = dict[Union[Pos, Trail], ConstructionState]
"""See :class:`motion_dict in ProcessState<ProcessState>`


*Type*: (:obj:`dict` [:obj:`Union` [:obj:`~type_aliases.Pos`, :obj:`~type_aliases.Trail`], :class:`~construction_state.ConstructionState`])"""

BuildingInstructions: TypeAlias = dict[Trail, BuildingInstruction]
"""Dictionary containing trails pointing to their :class:`~building_instruction.BuildingInstruction`.

*Type*: (:obj:`dict` [:obj:`~type_aliases.Trail`, :class:`~building_instruction.BuildingInstruction`])"""

Predecessors: TypeAlias = dict[Union[Pos, tuple[Pos, int, Pos]], Predecessor]
"""Dictionary containing node positions pointing to their predecessors. 

*Type*: :obj:`dict` [:obj:`Union` [:obj:`~type_aliases.Pos`, :obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`int`, :obj:`~type_aliases.Pos`]], :class`Predecessor`]"""