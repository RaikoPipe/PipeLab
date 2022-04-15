import numpy as np

import path_finding.restriction_functions
from path_finding.common_types import Pos


def get_optimal_attachment_pos(state_grid: np.ndarray, pos: Pos, part_id: int, direction: Pos):
    # todo: refactor
    countList = []
    sorter = [int(part_id / 2), 3, 1, 2, 4, 0, 5, 6]
    for i in range(0, part_id):
        countList.append(i)
    left = (-direction[1], -direction[0])
    right = (direction[1], direction[0])
    # first best option: both sides are empty
    for i in sorter:  # check pos right of pipe
        if i not in countList:
            continue
        n_left = (left[0] + (direction[0] * (i)), left[1] + (direction[1] * (i)))
        b_left = (pos[0] + n_left[0], pos[1] + n_left[1])
        n_right = (right[0] + (direction[0] * (i)), right[1] + (direction[1] * (i)))
        b_right = (pos[0] + n_right[0], pos[1] + n_right[1])
        if not path_finding.restriction_functions.out_of_bounds(b_left,
                                                                state_grid) and not path_finding.restriction_functions.out_of_bounds(
                b_right, state_grid):
            if state_grid[b_left] != 0 or state_grid[b_right] != 0:
                continue
            else:
                rec_att_pos = (pos[0] + i * direction[0] + 1, pos[1] + i * direction[1] + 1)
                return rec_att_pos
        else:
            rec_att_pos = (pos[0] + i * direction[0] + 1, pos[1] + i * direction[1] + 1)
            return rec_att_pos
    else:  # second best option: one side is empty
        for i in sorter:  # check pos right of pipe
            if i not in countList:
                continue
            n_left = (left[0] + (direction[0] * (i)), left[1] + (direction[1] * (i)))
            b_left = (pos[0] + n_left[0], pos[1] + n_left[1])
            n_right = (right[0] + (direction[0] * (i)), right[1] + (direction[1] * (i)))
            b_right = (pos[0] + n_right[0], pos[1] + n_right[1])
            if not path_finding.restriction_functions.out_of_bounds(b_left,
                                                                    state_grid) and not path_finding.restriction_functions.out_of_bounds(
                    b_right, state_grid):
                if state_grid[b_left] != 0 and state_grid[b_right] != 0:
                    continue
                else:
                    rec_att_pos = (pos[0] + i * direction[0] + 1, pos[1] + i * direction[1] + 1)
                    break
            else:
                rec_att_pos = (pos[0] + i * direction[0] + 1, pos[1] + i * direction[1] + 1)
                break
        else:
            rec_att_pos = pos
        return rec_att_pos
