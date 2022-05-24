from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from type_dictionary.class_types import *


@dataclass(slots=True)
class Solution:
    """Dataclass that contains information about the solution to a path problem."""

    rendering_dict: RenderingDict
    """Dict containing necessary information for rendering objects.

    *Type*: (:obj:`~type_aliases.RenderingDict`) """
    #:
    node_trail: NodeTrail
    """Dict containing all trail positions pointing to the part they are occupied by.

    *Type*: (:obj:`~type_aliases.NodeTrail`) """

    ordered_trails: OrderedTrails
    """List containing trails of a path ordered from start to goal.

    *Type*: (:obj:`~type_aliases.OrderedTrails`) """

    state_grid: StateGrid
    """See :obj:`~type_aliases.StateGrid`"""

    score: Optional[float]
    """Measure of solution quality. The lower the score the better the solution.

    *Type*: (:obj:`Optional` [:obj:`float`])"""

    algorithm: str
    """Algorithm that was used to search.

    *Type*: (:obj:`str`)"""

    path_problem: PathProblem
    """Path Problem that was solved

    *Type*: (:class:`PathProblem<path_problem>`)"""

    part_stock: dict[int, int]
    """The amount of parts still available after applying the solution.

    *Type*: (:class:`PathProblem<path_problem>`)"""

    node_path: NodePath = field(default_factory=list)
    """List of Positions with part IDs used (to get to this position).

    *Type*: (:obj:`~type_aliases.NodePath`)"""

