from dataclasses import dataclass


@dataclass
class Weights:
    """Dataclass that contains values for the weights used in a multi criteria a* search."""
    path_length: float  # Value of impact distance between nodes has on the score calculation
    cost: float  # Value of impact the cost of used parts have on the score calculation
    distance_to_obstacles: float  # value of impact of spacing between nodes/parts and obstacles
