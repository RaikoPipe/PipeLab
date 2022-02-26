from typing import NewType
#consider implementing with NewType

# todo: differentiate between Node and Pos! Node == (Pos,state), Pos = 2d-coordinates

Pos = tuple[int, int] # 2d coordinates of a node or distance between 2 nodes (relative pos)
Node = tuple[Pos,int] # node contains pos and a state
Path = list[Pos] # ordered list of Pos of Nodes that mark the placement position of pipe parts (in horizontal/vertical direction)
DefinitePath = list[(Pos,int)] # path that assigns each pos a part_id, or None if ambiguous
Trail = list[Pos] # an extension of a path, that contains each node visited by a path in order (in horizontal/vertical direction)
Connection = (Pos, Pos) # Two pos with a relationship
Layouts = set(DefinitePath) # a set containing multiple DefinitePaths