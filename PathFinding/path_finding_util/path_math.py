from __future__ import annotations

from math import copysign

from type_dictionary.common_types import *
from type_dictionary.constants import *


def sum_pos(a: Pos, b: Pos) -> Pos:
    """Sums up two pos element wise."""
    return a[0] + b[0], a[1] + b[1]


def diff_pos(a: Pos, b: Pos) -> Pos:
    """Subtracts two pos element wise. -> relative distance between two nodes"""
    return b[0] - a[0], b[1] - a[1]


def sum_abs_pos(a: Pos, b: Pos) -> Pos:
    """Sums up two absolute pos element wise."""
    a = abs(a[0]), abs(a[1])
    b = abs(b[0]), abs(b[1])
    return a[0] + b[0], a[1] + b[1]


def sum_absolute_a_b(a: int, b: int) -> int:
    """Sums up absolute a and absolute b."""
    a = abs(a)
    b = abs(b)
    return a + b


def diff_absolute_a_b(a: int, b: int) -> int:
    """Sums up absolute a and absolute b."""
    a = abs(a)
    b = abs(b)
    return a - b


def get_length_same_axis(a: Pos, b: Pos) -> int:
    """return length and of a and b if row xor column are the same"""

    if a[0] == b[0]:
        length = abs(a[1] - b[1])
    elif a[1] == b[1]:
        length = abs(a[0] - b[0])
    else:
        return -1

    return length


def get_rel_dist_same_axis(a: Pos, b: Pos) -> tuple:
    """return relative distance of a to b if row xor column are the same"""

    if a[0] == b[0]:
        distance = a[1] - b[1]
        rel_dist = (distance, 0)
    elif a[1] == b[1]:
        distance = a[0] - b[0]
        rel_dist = (0, distance)
    else:
        return -1, -1

    return rel_dist


# def get_adjacency(a: Pos, b: Pos) -> dict[str, bool, str, Pos]:
#     """Checks if two pos are adjacent and the direction b is from a."""
#     for rel_pos in valid_directions:
#         b_check = sum_pos(b, rel_pos)
#         adjacent = a == b_check
#         if adjacent:
#             adjacency = {"adj": adjacent, "dir": rel_pos}
#             return adjacency
#
#     return False


def manhattan_distance(a: tuple[int, int], b: tuple[int, int]):
    """Calculates the distance between to nodes in horizontal/vertical steps required.
    :param a: Position of a node.
    :param b: Position of another node."""
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    return distance


def get_direction(pos: Pos) -> Pos:
    """Returns the direction of a relative position.
    :param pos: Relative position"""
    x = pos[0]
    y = pos[1]

    if x == 0:
        y = y ** 0
    elif y == 0:
        x = x ** 0

    x = int(copysign(x, pos[0]))
    y = int(copysign(y, pos[1]))

    return x, y
