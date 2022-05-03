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
"""Contains each node visited (in a path for example). In specific contexts also called a layout."""

AbsoluteTrail = dict[Pos, int]
"""Trail where each position points to a part ID."""

DirectedConnection = tuple[Pos, Pos]
"""Two node positions in an ordered relationship."""

UndirectedConnection = frozenset
"""Two node positions in a non-directed relationship."""

StateGrid = np.ndarray
"""
A grid array where each position in the grid points to an integer value representing a state.
States:
0: Free position
1: occupied by obstacle
2: occupied by part
3: transition
"""
