from typing import NewType
#consider implementing with NewType

# todo: differentiate between Node and Pos! Node == (Pos,state), Pos = 2d-coordinates

Pos = tuple[int, int] # 2d coordinates of a node or distance between 2 nodes (relative pos)
Node = tuple[Pos,int] # node contains pos and a state
Path = list[Pos] # list of Pos of Nodes
Connection = (Pos, Pos) # Two pos with a relationship