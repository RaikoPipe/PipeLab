

from copy import deepcopy
from typing import Optional

from PathFinding.pf_data_class.PathProblem import PathProblem
from PathFinding.pf_data_class.Solution import Solution
from PathFinding.search_algorithm import find_path

completed_solutions_stack = []
"""List containing solved path problems."""


def check_solution_stack(path_problem: PathProblem) -> Optional[Solution]:
    """Checks if this path problem was already solved and returns the solution to it.
    Args:
        path_problem (:class:`PathProblem`): Path problem to solve.

    Returns:
        :class`Solution` if found in stack, else None.
        """

    for solution in completed_solutions_stack:
        solution: Solution
        same_state_grid = path_problem.state_grid == solution.path_problem.state_grid
        same_start_pos = path_problem.start_pos == solution.path_problem.start_pos
        same_goal = path_problem.goal_pos == solution.path_problem.goal_pos

        enough_parts = False
        if same_state_grid.all() and same_start_pos and same_goal:
            enough_parts = True
            parts = deepcopy(path_problem.part_stock)

            for node in solution.node_path:
                parts[node[1]] -= 1
                if parts[node[1]] < 0:
                    enough_parts = False
                    break

        if enough_parts:
            return solution
        else:
            return None


def get_solution(path_problem: PathProblem, draw_debug: bool = False) -> Optional[Solution]:
    """Wraps functions :func:`check_solution_stack` and find_path to look for a solution to the given path problem.

    Args:
        path_problem (:class:`PathProblem`): Path problem to solve.
        draw_debug(:obj:`bool`): See draw_debug in :obj:`find_path<search_algorithm>`

    Returns:
        :class:`Solution` when a solution was found, else :obj:`None`.
    """
    solution = check_solution_stack(path_problem=path_problem)
    if solution:
        return solution
    else:
        solution = find_path(path_problem, draw_debug)
        completed_solutions_stack.append(solution)
        return solution
