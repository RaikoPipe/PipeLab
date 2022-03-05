def grid_check(self, captured_state_grid: np.ndarray, parts_used: list, path: Path) -> Optional[Solution]:
    """Checks for changes in the captured grid and returns a solution on change (if solvable)."""
    if grid_changed(captured_state_grid, self.latest_path_problem.state_grid):
        self.latest_path_problem = self.get_new_path_problem(state_grid=captured_state_grid, parts_used=parts_used,
                                                             path=path)

        self.latest_state = self.get_current_layout_solution(captured_state_grid, parts_used, path)

        solution = check_solution_stack(self.completed_solutions_stack, self.latest_path_problem)
        if solution is None:
            solution = get_new_solution(path_problem=self.latest_path_problem, weights=self.weights)
        return solution

    return None  # grid has not changed, nothing to do.


def get_new_path_problem(self, state_grid: np.ndarray, parts_used: list, path: list) -> PathProblem:
    """Makes a copy of the initial path problem and adjusts it according to parameters. Returns the adjusted copy."""

    new_path_problem = deepcopy(self._initial_path_problem)  # make a copy of the original path problem

    current_pipe_stock = get_current_part_stock(copy(self._initial_path_problem.part_stock), parts_used)
    new_path_problem.part_stock = current_pipe_stock
    new_path_problem.state_grid = state_grid
    new_path_problem.start_node = path[-1]  # last entry is new start pos
    new_path_problem.start_direction = get_direction(
        diff_pos(path[-2], path[-1]))  # direction from second last entry to last entry is start direction
    # goal parameters stay the same

    return new_path_problem


def get_current_layout_solution(self, captured_state_grid: np.ndarray, parts_used: list, path: list):
    new_layout_solution = Solution(path=path, algorithm=None, parts=parts_used, path_problem=self._initial_path_problem,
                                   problem_solved=False, score=None, solution_grid=captured_state_grid)
    return new_layout_solution


def get_invalid_solutions(self, remove: bool = True) -> list:
    """Gets invalid solutions and returns them. Optionally also removes them. Should be called after modifying the
     path problem."""