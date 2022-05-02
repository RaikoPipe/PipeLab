from __future__ import annotations
import numpy as np

Pos = tuple[int, int]
"""2d coordinates/position of a node."""

Node = tuple[Pos, int]
"""Node contains 2D coordinates and an ID (state or part ID)."""

Path = list[Pos]
"""Ordered list of node positions from start to goal."""

NodePath = list[Node]
"""Ordered list of nodes from start to goal."""

Trail = tuple[Pos]
"""Contains each node visited (in a path for example)."""

AbsoluteTrail = dict[Pos,int]
"""Trail where each position points to a part ID."""

DirectedConnection = tuple[Pos, Pos]
"""Two node positions in an ordered relationship."""

UndirectedConnection = frozenset
"""Two node positions in a non-directed relationship."""

Layouts = list[Trail]
"""List containing trails in order from start to goal."""

StateGrid = np.ndarray
