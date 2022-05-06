from __future__ import annotations

import numpy as np
from typing import TypeAlias, Union


Pos = tuple[int, int]
"""Position/2D-Coordinates of a node. Also used as a direction or distance relative to a node.

Type: (:obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`int`])"""

Node: TypeAlias = tuple[Pos, int]
"""Node contains 2D coordinates and an ID (state or part ID).

Type: (:obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`int`])"""

Path: TypeAlias = list[Pos]
"""Ordered list of node positions from start to goal.

Type: (:obj:`list` [:obj:`~type_aliases.Pos`])"""

NodePath: TypeAlias = list[Node]
"""Ordered list of nodes from start to goal.

Type: (list[:obj:`~type_aliases.Node`])"""

Trail: TypeAlias = tuple[Pos]
"""Contains each node visited in a portion of a path or of a complete path. In specific contexts also called a layout.

Type: (:obj:`tuple` [:obj:`~type_aliases.Node`])"""

NodeTrail: TypeAlias = dict[Pos, int]
"""Trail where each position points to a part ID.

Type: (:obj:`dict` [:obj:`~type_aliases.Pos`, :obj:`int`])"""

DirectedConnection: TypeAlias = tuple[Pos, Pos]
"""Two node positions in an ordered relationship.

Type: (:obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`~type_aliases.Pos`])"""

StateGrid: TypeAlias = np.ndarray
"""
A grid array where each position in the grid points to an integer value representing a state.

States:
0: Free position
1: occupied by obstacle
2: occupied by part
3: transition

Type: (:obj:`numpy.ndarray`)
"""

PartStock: TypeAlias = dict[int,int]

MotionEvent = tuple[Union[Pos, int], int]
"""Tuple containing either a position for placement events or a part id for picking events, depending on the event code.

*Event Codes*:\n
1 -> fitting placed\n
2 -> pipe placed\n
3 -> attachment placed\n
4 -> accepted part from picking robot\n
5 -> picked part manually #unused

*Type*: (:obj:`tuple` [:obj:`Union` [:obj:`~type_aliases.Pos`, :obj:`int`], :obj:`int`])
"""

Action: TypeAlias = tuple[Pos, int, int]
"""Tuple containing the position, part id and event code of the next recommended building action.


*Type*: (:obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`int`, :obj:`int`])"""

NodePair = tuple[Pos, Union[Pos, tuple]]
"""Tuple containing two node positions in a relationship to each other (or alternatively only one node position).

*Type*: (:obj:`tuple` [:obj:`~type_aliases.Pos`, :obj:`Union` [:obj:`~type_aliases.Pos`, :obj:`tuple`]])"""

NodePairSet: TypeAlias = set[NodePair]
"""A set containing node pairs.

*Type*: (:obj:`set` [:obj:`~type_aliases.NodePair`])
"""

OrderedTrails: TypeAlias = tuple[Trail]
"""List containing trails, that are ordered from start to goal.

*Type*: (:obj:`list` [:obj:`~type_aliases.Trail`])"""

TrailSet: TypeAlias = set[Trail]
"""Set containing trails.

*Type*: (:obj:`set` [:obj:`~type_aliases.Trail`])"""

DirectionDict: TypeAlias = dict[Pos, Pos]
"""Dictionary containing positions that points to a direction.

*Type*: (:obj:`dict` [:obj:`~type_aliases.Pos`, :obj:`~type_aliases.Pos`])"""

FittingDirections: TypeAlias = dict[Pos, set[Pos]]
"""Dictionary that contains positions that point to multiple directions.

*Type*: (:obj:`dict` [:obj:`~type_aliases.Pos`, :obj:`set` [:obj:`~type_aliases.Pos`]])"""

PosSet = set[Pos]
"""A set containing node positions.

*Type*: (:obj:`set` [:obj:`~type_aliases.Pos`])"""

RenderingDict: TypeAlias = dict[int, Node]
"""

*Type*: (:obj:`dict` [:obj:`int`, :obj:`~type_aliases.Node`])"""
