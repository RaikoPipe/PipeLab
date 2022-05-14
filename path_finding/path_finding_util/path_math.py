from __future__ import annotations

from math import copysign

from type_dictionary.type_aliases import *


def sum_pos(a: Pos, b: Pos) -> Pos:
    """Sums up a and b elementwise.

    Args:
        a(:obj:`~type_aliases.Pos`)
        b(:obj:`~type_aliases.Pos`)
    Returns:
        (:obj:`~type_aliases.Pos`)
    """
    return a[0] + b[0], a[1] + b[1]


def diff_pos(a: Pos, b: Pos) -> Pos:
    """Subtracts two pos element wise.

    Args:
        a(:obj:`~type_aliases.Pos`)
        b(:obj:`~type_aliases.Pos`)
    Returns:
        (:obj:`~type_aliases.Pos`) -> Relative distance between two nodes.
    """
    return b[0] - a[0], b[1] - a[1]


def sum_abs_pos(a: Pos, b: Pos) -> Pos:
    """Sums up two absolute pos element wise.

    Args:
        a(:obj:`~type_aliases.Pos`)
        b(:obj:`~type_aliases.Pos`)
    Returns:
        (:obj:`~type_aliases.Pos`)
    """
    a = abs(a[0]), abs(a[1])
    b = abs(b[0]), abs(b[1])
    return a[0] + b[0], a[1] + b[1]

def get_length_same_axis(a: Pos, b: Pos) -> int:
    """return length of a and b if row xor column are the same

    Args:
        a(:obj:`~type_aliases.Pos`)
        b(:obj:`~type_aliases.Pos`)
    Returns:
        (:obj:`int`)
    """

    if a[0] == b[0]:
        length = abs(a[1] - b[1])
    elif a[1] == b[1]:
        length = abs(a[0] - b[0])
    else:
        return -1

    return length


def get_rel_dist_same_axis(a: Pos, b: Pos) -> Pos:
    """Returns relative distance of a to b if row xor column are the same.

    Args:
        a(:obj:`~type_aliases.Pos`)
        b(:obj:`~type_aliases.Pos`)
    Returns:
        (:obj:`~type_aliases.Pos`)
    """

    if a[0] == b[0]:
        distance = a[1] - b[1]
        rel_dist = (distance, 0)
    elif a[1] == b[1]:
        distance = a[0] - b[0]
        rel_dist = (0, distance)
    else:
        return -1, -1

    return rel_dist

def manhattan_distance(a: Pos, b: Pos):
    """Calculates the distance between to nodes in horizontal/vertical steps required.

    Args:
        a(:obj:`~type_aliases.Pos`)
        b(:obj:`~type_aliases.Pos`)
    Returns:
        (:obj:`~type_aliases.Pos`)"""
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    return distance


def get_direction(pos: Pos) -> Pos:
    """Returns the direction of a relative position.
    Args:
         pos(:obj:`~type_aliases.Pos`): Some relative position

    Returns:
        (:obj:`~type_aliases.Pos`)
        """
    x = pos[0]
    y = pos[1]

    if x == 0:
        y = y ** 0
    elif y == 0:
        x = x ** 0

    x = int(copysign(x, pos[0]))
    y = int(copysign(y, pos[1]))

    return x, y
