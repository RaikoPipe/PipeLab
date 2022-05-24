from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from path_finding.pf_data_class.weights import Weights
from type_dictionary.type_aliases import *


@dataclass(slots=True) # slots increase property access performance by ~20% compared to dictionaries.
class PathProblem:
    """Describes a path problem to be solved by the :ref:`Solution Manager Module <Solution Manager Module>`.
    """

    state_grid: StateGrid  #:
    """See :obj:`~type_aliases.StateGrid`"""

    start_pos: Pos
    """
    Start position of the search.
    
    *Type*: (:obj:`~type_aliases.Pos`) 
    """

    goal_pos: Pos
    """
    Goal position that needs to be reached
    
    *Type*: (:obj:`~type_aliases.Pos`) 
    """

    start_directions: set[Pos]
    """
    Set containing directions that restrict in which direction the search algorithm can start the search.
    
    *Type*: (set[:obj:`~type_aliases.Pos`]) """

    goal_directions: set[Pos]
    """
    Set containing directions that restrict in which direction the search algorithm can append to the goal.
    
    *Type*: (set[:obj:`~type_aliases.Pos`])
    """
    transition_points: set[Pos]
    """
    Set containing transition points that show where a transition from one wall to another occurs.
    
    *Type*: (set[:obj:`~type_aliases.Pos`]) 
    """

    starting_part: Optional[int]
    """
    Part ID that is placed at the start point (Error will be raised, if this is not a fitting ID, since this is a 
    placeholder for potential features implemented in the future)
    
    *Type*: (:obj:`Optional` [:obj:`int`]) 
    """

    part_stock: PartStock
    """
    Part IDs pointing the amount of parts available for assembly.
    
    *Type*: (set[:obj:`~type_aliases.PartStock`]) 
    """

    part_cost: dict[int, float]
    """
    Part IDs pointing to their cost value.
    
    *Type*: (:obj:`dict` [:obj:`int`, :obj:`float`]) 
    """

    weights: Optional[Weights]
    """
    Weights used if search algorithm is a multi-criteria search algorithm (mca*, mcsa*)
    
    *Type*: (:obj`Optional` [:class:`weights.Weights`])
    """

    algorithm: str = "mcsa*"
    """
    Defines how scores are calculated and therefore how nodes are evaluated in the search.
    Accepted algorithms:
    mcsa*
    mca*
    dijkstra
    best-first
    
    *Type*: (:obj: `str`) 
    """
