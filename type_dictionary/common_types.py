from __future__ import annotations

import numpy as np
from typing import TypeAlias


Pos: TypeAlias = tuple[int, int]
"""2d coordinates/position of a node."""

Node: TypeAlias = tuple[Pos, int]
"""Node contains 2D coordinates and an ID (state or part ID)."""

Path: TypeAlias = list[Pos]
"""Ordered list of node positions from start to goal."""

NodePath: TypeAlias = list[Node]
"""Ordered list of nodes from start to goal."""

Trail: TypeAlias = tuple[Pos]
"""Contains each node visited (in a path for example). In specific contexts also called a layout."""

NodeTrail: TypeAlias = dict[Pos, int]
"""Trail where each position points to a part ID."""

DirectedConnection: TypeAlias = tuple[Pos, Pos]
"""Two node positions in an ordered relationship."""

UndirectedConnection: TypeAlias = frozenset
"""Two node positions in a non-directed relationship."""

StateGrid: TypeAlias = np.ndarray
"""
A grid array where each position in the grid points to an integer value representing a state.
States:
0: Free position
1: occupied by obstacle
2: occupied by part
3: transition
"""
