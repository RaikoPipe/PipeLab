from path_finding.restriction_functions import diff_pos
from path_finding.path_math import get_length_same_axis, get_direction
from path_finding.common_types import *


def is_between(a:Pos, b: Pos, pos:Pos) -> bool:
    #todo: change frozenset to tuple, because iterating through frozenset might be slow

    fit_dir = get_direction(diff_pos(a,b))
    con_dir = get_direction(diff_pos(pos, a))

    return fit_dir == con_dir




