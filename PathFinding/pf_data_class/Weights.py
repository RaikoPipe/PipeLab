from dataclasses import dataclass


@dataclass(slots=True)
class Weights:
    """Dataclass that contains values for the weights used in a multi criteria a* search."""
    path_length: float
    """Value of impact distance between nodes has on the score calculation.
    
    Type: :obj:`float`"""
    cost: float
    """Value of impact the cost of assembled parts have on the score calculation.
    
    Type: :obj:`float`"""
    distance_to_obstacles: float  #
    """Value of impact of spacing between nodes/parts and obstacles.
    
    Type: :obj:`float`"""
