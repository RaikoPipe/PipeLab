from typing import NewType
#consider implementing with NewType

Pos = tuple[int, int] # 2d coordinates of a node or distance between 2 nodes (relative pos)
Node = tuple[Pos,int] # node contains pos and a state/part_id
Path = list[Pos] # ordered list of Pos of Nodes that mark the placement position of pipe parts (in horizontal/vertical direction)
DefinitePath = list[tuple[Pos,int]] # path that assigns each pos a part_id, or None if ambiguous
Trail = list[Pos] # an extension of a path, that contains each node visited by a path in order (in horizontal/vertical direction)
Positions = set[Pos]
DirectedConnection = tuple[Pos, Pos] # Two pos with an ordered relationship
UndirectedConnection = frozenset() # two pos with an unordered relationship
Layouts = set(DefinitePath) # a set containing multiple DefinitePaths
