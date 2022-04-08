def grid_check(self, captured_state_grid: np.ndarray, parts_used: list, path: Path) -> Optional[Solution]:
    """Checks for changes in the captured grid and returns a solution on change (if solvable)."""
    if grid_changed(captured_state_grid, self.latest_path_problem.state_grid):
        self.latest_path_problem = self.get_new_path_problem(state_grid=captured_state_grid, parts_used=parts_used,
                                                             path=path)

        self.latest_state = self.get_current_layout_solution(captured_state_grid, parts_used, path)

        solution = check_solution_stack(self.completed_solutions_set, self.latest_path_problem)
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
    new_path_problem.start_pos = path[-1]  # last entry is new start pos
    new_path_problem.start_directions = get_direction(
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

def event_received_robot_state(self, state_id):
    # todo: determine next step
    print("not implemented")

def event_captured_state_grid(self, captured_state_grid):
    # todo: interpret new data, exec actions as needed, update latest path problem, update current_layout_solution
    if event_interpreting.grid_changed(latest_state_grid=self.latest_state.state_grid,
                                       captured_state_grid=captured_state_grid):

        event = event_interpreting.check_action_event(picked_parts=self.picked_parts,
                                                      placed_parts=self.placed_parts,
                                                      latest_state_grid=self.latest_path_problem.state_grid,
                                                      captured_state_grid=captured_state_grid)
        # todo: on event 1: check if path of current state overlaps with optimal solution ->
        if event["code"] == -1:
            # recalculate path of current state
            trail_list = event_interpreting.get_trails_from_state_grid(state_grid=captured_state_grid,
                                                                       searched_state=2)
            for trail in trail_list:
                path = event_interpreting.get_path_from_trail(trail)
                definite_path = event_interpreting.get_definite_path_from_path(path=path,
                                                                               part_stock=self._initial_path_problem.part_stock)

            pass
        elif event["code"] == 1:
            # todo: check where part was removed, change latest_state.definite_path, check if we are back on optimal solution

            pass
        elif event["code"] == 2:
            # todo: check where part has been placed and if it is adjacent to a current layout and valid -> expand this path
            # todo: check if paths have been connected: fuse into one path
            # todo: check if current path overlaps with optimal solution
            pass
        else:
            print("Unknown Error occurred!")

def update_current_state(self, state_grid: np.ndarray, path: DefinitePath):
    self.latest_state.state_grid = state_grid
    self.latest_state.definite_path = path

def event_part_id_picked(self, part_id_picked):
    self.picked_parts.append(part_id_picked)
    # todo: highlight placement options in visualization