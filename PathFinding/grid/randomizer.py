import random
from copy import deepcopy

import numpy as np

from TypeDictionary.type_aliases import StateGrid


def set_random_obstacles(probability: float, state_grid: StateGrid) -> np.ndarray:
    """Randomly changes nodes on the state grid to 1 according to the frequency.

    Args:
        probability(obj:`float`): probability of a node on the state grid receiving the state 1 (obstacle)
        state_grid(obj:`~type_aliased.StateGrid`): An empty state grid.
    """
    new_state_grid = deepcopy(state_grid)

    for pos, _ in np.ndenumerate(state_grid):
        if random.random() < probability:
            new_state_grid[pos] = 1

    return new_state_grid
