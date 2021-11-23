from dataclasses import dataclass

@dataclass
class PathProblem:
    """Data class that contains detailed information about the path problem."""
    state_grid: any
    start_node: tuple
    goal_node: tuple

    start_axis: tuple # direction that start is restricted to
    goal_axis: tuple # direction that goal is restricted to (i.e. last part that is linked to the goal)

    goal_is_transition: bool # direction of the transition point; Is None if goal is not a transition
    pipe_stock: dict #{point_length: amount}
    part_cost: dict # dictionary that contains the replacement costs (or opportunity costs) of parts

@dataclass
class Solution:
    """Data class that contains information about a solution and the problem it solved."""
    path: list # the calculated path
    parts: list # what parts need to be used to build layout
    solution_grid : any # state grid with inserted solution layout
    score: float # how good the solution is -> lower is better
    algorithm: str # what algorithm has been used to search
    path_problem: PathProblem # the original path problem

@dataclass
class Weights:
    """Data class that contains values for the weights used in a multi criteria a* search."""
    path_length : float
    cost: float
    distance_to_obstacles: float