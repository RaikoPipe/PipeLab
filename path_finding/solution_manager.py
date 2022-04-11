from path_finding.search_algorithm import find_path
from data_class.PathProblem import PathProblem
from data_class.Solution import Solution
from typing import Optional

completed_solutions_list = []

def check_solution_stack(path_problem: PathProblem) -> Optional[Solution]:
    # todo: add required_parts to Solution, check if part_stock >= required_parts
    # fixme: check for all available solutions: has same state grid, has same start/goal pos, has enough parts
    for solved_path_problem in completed_solutions_list:
        if path_problem == solved_path_problem:
            return solved_path_problem
        else:
            return None

def get_solution(path_problem: PathProblem, draw_debug: bool = False) -> Solution:
    solution = check_solution_stack(path_problem=path_problem)
    if solution:
        return solution
    else:
        solution = find_path(path_problem, draw_debug)
        completed_solutions_list.append(solution)
        return solution

# todo: add function to test every other algorithm if chosen algorithm failed producing a solution