from typing import Optional

from PathFinding.data_class.PathProblem import PathProblem
from PathFinding.data_class.Solution import Solution
from PathFinding.search_algorithm import find_path

completed_solutions_stack = []


def check_solution_stack(path_problem: PathProblem) -> Optional[Solution]:
    """Returns a solution if one exists that solves the given path problem."""
    # todo: add required_parts to Solution, check if part_stock >= required_parts
    # fixme: check for all available solutions: has same state grid, has same start/goal pos, has enough parts
    for solved_path_problem in completed_solutions_stack:
        if path_problem == solved_path_problem:
            return solved_path_problem
        else:
            return None


def get_solution(path_problem: PathProblem, draw_debug: bool = False) -> Optional[Solution]:
    """Returns a solution to the path problem, if solvable."""
    solution = check_solution_stack(path_problem=path_problem)
    if solution:
        return solution
    else:
        solution = find_path(path_problem, draw_debug)
        completed_solutions_stack.append(solution)
        return solution
