from path_finding.search_algorithm import find_path
from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
from typing import Optional

completed_solutions_stack = {}

def check_solution_stack(path_problem: PathProblem) -> Optional[Solution]:
    # todo: add required_parts to Solution, check if part_stock >= required_parts
    for solved_path_problem in completed_solutions_stack:
        if path_problem == solved_path_problem:
            return solved_path_problem
        else:
            return None

def get_solution(path_problem: PathProblem) -> Solution:
    solution = check_solution_stack(path_problem=path_problem)
    if solution:
        return solution
    else:
        return find_path(path_problem)
