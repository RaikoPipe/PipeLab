from dataclasses import dataclass


@dataclass
class Weights:
    """Data class that contains values for the weights used in a multi criteria a* search."""
    path_length : float
    cost: float
    distance_to_obstacles: float