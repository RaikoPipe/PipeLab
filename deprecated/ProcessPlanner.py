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


def handle_new_input(self, worker_event: tuple[Optional[int], Pos], pick_events: list) -> bool:
    """evaluates worker event (and event pos on the mounting wall) to determine the current construction layout.
    :returns deviation occurrence"""
    # fixme: deepcopy might be slow, consider alternative



    deviation_occurred = False

    if pick_events:
        for part_id in pick_events:
            self.tentative_state.picked_parts[part_id] += 1

    event_code = worker_event[0]
    event_pos = worker_event[1]

    if worker_event[0]:

        if event_code == 2:
            self.tentative_state.motion_pipe_pos.add(event_pos)
        elif event_code == 3:
            self.tentative_state.motion_attachment_pos.add(event_pos)

        #todo: check for removals, update tentative state

        if self.deviation_event(motion_events=worker_event, tentative_state=self.tentative_state):
            if self.tentative_state.deviated_from_opt_sol:

                if not deviated_from_path(current_state=self.tentative_state, optimal_solution=self.optimal_solution):
                    self.tentative_state.deviated_from_opt_sol = False
                    # todo: notify worker?
                #todo: check if deviated from latest deviation solution
                elif deviated_from_path(current_state=self.tentative_state, optimal_solution=self.latest_deviation_solution):
                    deviation_occurred = True
                    # todo: create new partial solution
                    self.latest_deviation_solution = partial_solutionizer.find_partial_solution_simple(self.tentative_state.layouts,
                                                                                                       self.tentative_state.state_grid,
                                                                                                       self._initial_path_problem)
                    self.tentative_state.aimed_solution = self.latest_deviation_solution
            else:
                if deviated_from_path(current_state=self.tentative_state, optimal_solution=self.optimal_solution):
                    deviation_occurred = True
                    self.tentative_state.deviated_from_opt_sol = True
                    self.latest_deviation_solution = partial_solutionizer.find_partial_solution_simple(
                        self.tentative_state.layouts,
                        self.tentative_state.state_grid,
                        self._initial_path_problem)

        if deconstruction_event


    #todo: recognize pick events
    # detect if part was picked by worker: if picking event occurs outside robot state pick part


    if pick_events or worker_event:
        self.update_latest_state(old_state=self.latest_state, new_state=self.tentative_state)

    return deviation_occurred