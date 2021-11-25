import heapq

from path_finding import restrictions as rest
from path_finding.path_data_classes import Solution, PathProblem
from path_finding.restrictions import build_path, manhattan_distance, neighbor_restricted, get_m_score, get_e_score, \
    determine_neighbor_pos, get_f_score, get_worst_move_cost
from path_finding.tuple_math import *
from typing import Optional

# idea: make a separate function for search showcase
# optimization idea: save pipe stock in each node, save state grid in each node (reduce pipe stock, occupy path from current node to neighbor node on state grid)

class PathFinder:
    """class for calculating a path solution."""

    def __init__(self, path_problem: PathProblem):
        self._path_problem = path_problem # the original path problem
        self.state_grid = path_problem.state_grid
        self.start_node = path_problem.start_node
        self.goal_node = path_problem.goal_node
        self.start_axis = path_problem.start_axis
        self.goal_axis = path_problem.goal_axis
        self.pipe_stock = path_problem.pipe_stock
        self.goal_is_transition = path_problem.goal_is_transition
        self.part_cost = path_problem.part_cost
        self.worst_move_cost, self.worst_moves = get_worst_move_cost(self.part_cost)
        self.solutions = []
        # todo: give starting part as parameter

    def find_path(self, weights, algorithm) -> Optional[Solution]:
        """Searches for a solution for the given path problem."""

        closed_list = set()
        open_list = []

        predecessor_node = {}

        score_start = {self.start_node: 0}  # G
        upper_bound_distance = manhattan_distance(self.start_node,
                                                  self.goal_node)  # artificial upper bound for distance
        total_score = {self.start_node: upper_bound_distance}  # F

        heapq.heappush(open_list, (total_score[self.start_node], self.start_node))

        used_part = {}

        while open_list:
            current_node = heapq.heappop(open_list)[1]  # pops the node with the smallest score from open_list
            current_path = build_path(current_node=current_node, predecessor_node=predecessor_node,
                                      start_node=self.start_node)

            if current_node == self.start_node:
                verifiable_neighbors = determine_neighbor_pos(axis=self.start_axis,
                                                              goal_node=self.goal_node, goal_axis=self.goal_axis,
                                                              current_node=current_node,
                                                              current_path=current_path,
                                                              pipe_stock=self.pipe_stock, used_parts=used_part)
            else:
                axis = rest.get_direction_of_pos(diff_tuple(current_node, predecessor_node.get(current_node)))
                verifiable_neighbors = determine_neighbor_pos(axis=axis,
                                                              goal_node=self.goal_node, goal_axis=self.goal_axis,
                                                              current_node=current_node,
                                                              current_path=current_path,
                                                              pipe_stock=self.pipe_stock, used_parts=used_part)

            current_state_grid = rest.get_current_state_grid(current_path, self.state_grid)

            if current_node == self.goal_node:
                # search is finished!
                overall_score = total_score[current_node]
                solution_parts = []
                while current_node in used_part:
                    solution_parts.append(used_part[current_node])
                    current_node = predecessor_node[current_node]
                solution_parts = solution_parts[::-1]
                solution = Solution(current_path, solution_parts, current_state_grid, overall_score, algorithm,
                                    self._path_problem)

                self.solutions.append(solution)
                return solution

            closed_list.add(current_node)

            for (pos, part_id) in verifiable_neighbors:
                neighbor_node = sum_tuple(current_node, pos)

                current_score_start_distance = score_start[current_node] + \
                                               manhattan_distance(current_node, neighbor_node) / total_score[
                                                   self.start_node]

                if neighbor_node in closed_list:  # and current_score_start >= score_start.get(neighbor_node, 0):
                    continue

                if neighbor_restricted(current_node=current_node, neighbor_node=neighbor_node, pos=pos,
                                       current_state_grid=current_state_grid):
                    continue

                if current_score_start_distance < score_start.get(neighbor_node, 0) or neighbor_node not in [p[1] for p
                                                                                                             in
                                                                                                             open_list]:
                    predecessor_node[neighbor_node] = current_node

                    used_part[neighbor_node] = part_id

                    score_start[neighbor_node] = current_score_start_distance

                    current_score_goal_distance = get_m_score(algorithm=algorithm, goal_node=self.goal_node,
                                                              neighbor_node=neighbor_node,
                                                              weights=weights, upper_bound=total_score[self.start_node])

                    current_score_extra = get_e_score(algorithm=algorithm, weights=weights, current_node=current_node,
                                                      neighbor_pos=pos,
                                                      part_cost=self.part_cost, worst_move_cost=self.worst_move_cost,
                                                      current_state_grid=current_state_grid, part_id=part_id)

                    total_score[neighbor_node] = get_f_score(current_score_start_distance, current_score_goal_distance,
                                                             current_score_extra, total_score[current_node], algorithm)
                    heapq.heappush(open_list, (total_score[neighbor_node], neighbor_node))
        else:
            # no solution found!
            return None

    def get_invalid_solutions(self, remove:bool=True) -> list:
        """Gets invalid solutions and returns them. Optionally also removes them. Should be called after modifying the
         path problem."""

        #todo: code function

    def set_path_problem(self, path_problem: PathProblem, remove_invalid_solutions: bool=False) -> list:
        """Convenience function for modifying a path problem. Returns solutions that are now invalid."""

        self.state_grid = path_problem.state_grid
        self.start_node = path_problem.start_node
        self.goal_node = path_problem.goal_node
        self.start_axis = path_problem.start_axis
        self.goal_axis = path_problem.goal_axis
        self.pipe_stock = path_problem.pipe_stock
        self.goal_is_transition = path_problem.goal_is_transition
        self.part_cost = path_problem.part_cost
        self.worst_move_cost, self.worst_moves = get_worst_move_cost(self.part_cost)

        return self.get_invalid_solutions(remove=remove_invalid_solutions)


