from path_finding.common_types import *
from constants import *

def sum_pos(a:Pos, b:Pos) -> Pos:
    """Sums up two pos element wise."""
    return a[0] + b[0], a[1] + b[1]

def diff_pos(a:Pos, b:Pos) -> Pos:
    """Subtracts two pos element wise. -> relative distance between two nodes"""
    return b[0]-a[0], b[1]-a[1]

def sum_abs_pos(a:Pos, b:Pos) -> Pos:
    """Sums up two absolute pos element wise."""
    a = abs(a[0]),abs(a[1])
    b = abs(b[0]),abs(b[1])
    return a[0] + b[0], a[1] + b[1]

def sum_absolute_a_b(a: int, b: int) -> int:
    """Sums up absolute a and absolute b."""
    a = abs(a)
    b = abs(b)
    return a + b

def diff_absolute_a_b(a:int, b:int) -> int:
    """Sums up absolute a and absolute b."""
    a = abs(a)
    b = abs(b)
    return a - b

def get_adjacency(a:Pos, b:Pos)-> dict[str:bool,str: Pos]:
    """Checks if two pos are adjacent and the direction b is from a."""
    for rel_pos in directions:
        b_check = sum_pos(b,rel_pos)
        adjacent = a == b_check
        if adjacent:
            adjacency = {"adj": adjacent, "dir": rel_pos}
            return adjacency

    return False

    