import heapq
import numpy as np
from datetime import datetime
from path_finding import interpret_path as pint
from vpython import *

from path_finding.path_data_classes import Solution, PathProblem
from rendering import object_classes, positional_functions_old as lvf
from copy import deepcopy
import time
from path_finding.tuple_math import *

#todo: full refactoring
#save showcase inside a show_case_grid

#if v is below z or above z+1 then shift is not allowed
def crossingRestricted(v, neighbor, z):
    if neighbor[1] > z and v[1] < z:
        return True
    elif neighbor[1] < z+1 and v[1] > z+1:
        return True
    else:
        return False

#if neighbor vn is outside array shape, dont allow
def outOfBounds(vn, array):
    if 0 <= vn[0] < array.shape[0]:
        if 0 <= vn[1] < array.shape[1]:
            return False
        else:
            return True # array bound y walls
    else:
        return True # array bound x walls

#checks if nodes from v to vn collide obstacle
def collidedObstacle(v, n, array):
    nLength = abs(n[0] - n[1])
    axis = pint.getAxis(n)
    for i in range(1, nLength + 1):
        pos = (v[0] + axis[0] * i, v[1] + axis[1] * i)
        if array[pos] != 0:
            return True

#build the path (doesnt include s, but it should)
def buildPath(current_node, pointer_to_previous_node, start_pos):
    path = []
    while current_node in pointer_to_previous_node:
        path.append(current_node)
        current_node = pointer_to_previous_node[current_node]
    path.append(start_pos)
    path = path[::-1] # reverses the path to correct order (from start to goal)
    return path

#check if parts are available
def stockCheck(path, type_dict, part_dict):
    availableParts = pint.pipe_stock_check(path, type_dict, part_dict)

#changes Neighbors efficiently
def changeNeighbors(Neighbors, axis):
    changedNeighbors = {}
    for index, (n, t) in enumerate(Neighbors.items()):
        nAxis = pint.getAxis(n)
        if nAxis == (axis[0],axis[1]):
            changedNeighbors[(t*nAxis[0],t*nAxis[1])] = t
        else:
            changedNeighbors[n] = t
    return changedNeighbors

#changes neighbors if certain tuple needs to be added
def ChangeZNeighbors(Neighbors, add, axis):
    changedNeighbors = {}
    for index, (n, t) in enumerate(Neighbors.items()):
        nAxis = pint.getAxis(n)
        if nAxis == (axis.x,axis.y):
            changedNeighbors[(n[0], n[1] + add)] = t
        else:
            changedNeighbors[n] = t
    return changedNeighbors

#changes neighbors, if v is close to goal
def changeClosingNeighbors(Neighbors, axis, closingList):
    changedNeighbors = {}
    for index, (n, t) in enumerate(Neighbors.items()):
        nAxis = pint.getAxis(n)
        if nAxis == (axis.x,axis.y) and t in closingList:
            changedNeighbors[(t*nAxis[0],t*nAxis[1])] = t
        else:
            changedNeighbors[n] = t
    return changedNeighbors

#quite a complicated function for determining neighbor positions
def positionDependence(Neighbors, start, startAxis, goal,goalAxis, current):
    newNeighbors = deepcopy(Neighbors)
    if current == start:
        newNeighbors = changeNeighbors(Neighbors, startAxis)
    else:
        closingParts = []
        for index, (n, t) in enumerate(Neighbors.items()):
            axis = pint.getAxis(n)
            gAxis = (-goalAxis[0], -goalAxis[1])
            absGoalDistance = np.abs(np.abs(goal[0] - current[0]) - np.abs(goal[1]-current[1]))
            if (((current[0]+t*axis[0],current[1]+t*axis[1])==goal and axis == gAxis) and absGoalDistance == t)\
                    or ((current[0] + n[0], current[1] + n[1]) == goal and axis == gAxis):
                closingParts.append(t)
        if closingParts:
                newNeighbors = changeClosingNeighbors(Neighbors, -goalAxis, closingParts)

    return newNeighbors

#self explanatory
def manhattanDistance(a, b):
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    return distance

def getNeighbors(dict):
    neighbors = {}
    for key, (type, count) in enumerate(dict.items()):
        neighbors[(type+1,0)] = type
        neighbors[(-type-1, 0)] = type
        neighbors[(0,type+1)] = type
        neighbors[(0,-type-1)] = type
    return neighbors

def costMinO(Matrix, a, n):
    axis = pint.getAxis(n)
    nLength = abs(n[0] - n[1])
    MinO = nLength * 2
    upperBound = MinO
    left = (-axis[1], -axis[0])
    right = (axis[1], axis[0])
    for i in range(nLength): #check pos right of pipe
        n_left = (left[0]+(axis[0]*(i)),left[1]+(axis[1]*(i)))
        b_left = (a[0] + n_left[0], a[1] + n_left[1])
        if not outOfBounds(b_left,Matrix):
            if Matrix[b_left] != 0:
                continue
        MinO = MinO - 1
    for i in range(nLength): #check pos left of pipe
        n_right = (right[0]+(axis[0]*(i)),right[1]+(axis[1]*(i)))
        b_right = (a[0] + n_right[0], a[1] + n_right[1])
        if not outOfBounds(b_right,Matrix):
            if Matrix[b_right] != 0:
                continue
        MinO = MinO - 1
    return MinO/upperBound

def getMax(Dict):
    maxValue = 0
    for index, (x,y) in enumerate(Dict):
        if x > maxValue:
            maxValue = x
    return maxValue

def neighbor_restricted(current_node, neighbor_node, start_pos, pos, previous, current_state_grid) -> bool:

    if not outOfBounds(neighbor_node, current_state_grid):
        if collidedObstacle(current_node, pos, current_state_grid):
            return True

    else: return True

    return False

def get_f_score(algorithm, weights, current_node, neighbor_node, neighbor_pos, part_id, part_cost, current_state_grid, current_score_start,
                score_goal, start_pos, goal_pos):
    """Calculates the score of the current node according to the given algorithm"""
    score = 0
    if algorithm == "mca*":
        distance = (current_score_start +
                    manhattanDistance(neighbor_node, goal_pos)) / score_goal[start_pos] * weights.path_length

        # todo: suggestion: add a dict containing part id:length to explicitly differentiate between length and id

        if part_id == 0:
            length = 1
        else:
            length = part_id
        cost = length * part_cost[part_id] * weights.cost #calculates cost per passed nodes

        distance_to_obstacles = costMinO(current_state_grid, current_node, neighbor_pos)* weights.distance_to_obstacles

        score = distance + cost + distance_to_obstacles

    # todo: implement dijkstra, astar, bestfirst

    return score

def get_corner_neighbors(axis, available_parts):
    """Returns all the neighbors that are allowed as the next move by the currently available corner parts
     (usually only 1 neighbor)."""
    #corner moves can ONLY go in one direction
    neighbors = []
    if 0 in available_parts:
        neighbors.append((axis, 0))

    return neighbors

def get_pipe_neighbors(axis, available_parts, at_start) -> list:
    """Returns all neighbors that are allowed as the next move by the currently available pipe parts"""
    # pipes moves have two variants, depending on if the current axis
    neighbors = []
    for part_id in available_parts:
        # for all part IDs except corner the point length matches the id
        if part_id == 0:
            continue
        if at_start:
            # only allow neighbors that meet start condition
            neighbors.append(((part_id*axis[0], part_id*axis[1]), part_id))
        else:
            # only allow neighbors that meet corner condition
            neighbors.append(((part_id * axis[1], part_id * axis[0]), part_id))
            neighbors.append(((part_id * -axis[1], part_id * -axis[0]), part_id))

    return neighbors

def goal_restricted(part_id, neighbor_node, axis, goal_pos, goal_axis):
    """Checks if the axis restriction of the goal is violated."""
    if neighbor_node is not goal_pos:
        # no restriction check necessary, we have not reached the goal
        return False

    if part_id == 0:
        # check if corner axis is adjacent to goal axis
        if sum_absolute_a_b(axis[0], goal_axis[1]) != 0 or sum_absolute_a_b(axis[1], goal_axis[0]) != 0:
            return False
    else:
        # simply check if pipe axis is the same as goal axis
        if axis == goal_axis:
            return False

    return True

def determine_neighbor_pos(axis, goal_pos, goal_axis, current_node, predecessor_node, current_path,
                           pipe_stock, used_parts) -> list:
    """Determines what neighbors are reachable from the current position and with the available parts."""
    # check which parts are still available for the next move
    available_parts = pint.pipe_stock_check(current_path, pipe_stock, used_parts)
    neighbor_pos = []
    previous_part = used_parts.get(current_node)
    if previous_part is None:
        # todo: it is assumed that if there is no previous part, we are at start pos. However, this only works when
        #  nodes cant be overwritten while searching; proposal: check instead if current node is start (easy)
        # current node is start -> only one axis is allowed
        neighbor_pos.extend(get_corner_neighbors(axis, available_parts))
        neighbor_pos.extend(get_pipe_neighbors(axis, available_parts, True))
    elif previous_part == 0:
        # previous move was corner -> only pipes allowed
        neighbor_pos.extend(get_pipe_neighbors(axis, available_parts, False))
    else:
        # previous mode was pipe -> only corners allowed
        neighbor_pos.extend(get_corner_neighbors(axis, available_parts))

    for (pos, part_id) in neighbor_pos:
        if goal_restricted(part_id, sum_tuple(current_node, pos), axis, goal_pos, goal_axis):
            # disallow any neighbors that violate goal condition
            neighbor_pos.pop(pos)


    return neighbor_pos

# Todo: Improvement idea: Use overall score (of current path + neighbor) to measure score of a node.
#  This could be an opportunity to get the actual optimal solution (hard)
class PathFinder:
    """class for calculating a path solution"""
    def __init__(self, path_problem:PathProblem):
        self.path_problem = path_problem
        self.state_grid = path_problem.state_grid
        self.start_pos = path_problem.start_pos
        self.goal_pos = path_problem.goal_pos
        self.start_axis = path_problem.start_axis
        self.goal_axis = path_problem.goal_axis
        self.pipe_stock = path_problem.pipe_stock
        self.goal_is_transition = path_problem.goal_is_transition
        self.part_cost = path_problem.part_cost
    def find_path(self, weights, algorithm):
        """"""
        closed_list = set()
        open_list = []

        predecessor_node = {}

        score_start = {self.start_pos:0} # G
        score_goal = {self.start_pos: manhattanDistance(self.start_pos,self.goal_pos)} # F

        heapq.heappush(open_list, (score_goal[self.start_pos], self.start_pos))

        used_part = {}

        while open_list:
            current_node = heapq.heappop(open_list)[1] # pops the node with the smallest score from open_list
            current_path = buildPath(current_node, predecessor_node, self.start_pos)

            if current_node == self.start_pos:
                verifiable_neighbors = determine_neighbor_pos(axis=self.start_axis,
                                                              goal_pos=self.goal_pos, goal_axis=self.goal_axis,
                                                              current_node=current_node,
                                                              predecessor_node=predecessor_node.get(current_node),
                                                              current_path=current_path,
                                                              pipe_stock=self.pipe_stock, used_parts=used_part)
            else:
                axis = pint.getAxis(diff_tuple(current_node, predecessor_node.get(current_node)))
                verifiable_neighbors = determine_neighbor_pos(axis = axis,
                                                              goal_pos=self.goal_pos, goal_axis=self.goal_axis,
                                                              current_node=current_node,
                                                              predecessor_node=predecessor_node.get(current_node),
                                                              current_path=current_path,
                                                              pipe_stock=self.pipe_stock, used_parts=used_part)

            current_state_grid = pint.getAlteredMatrix(current_path, self.state_grid)

            if current_node == self.goal_pos:
                # search is finished!
                #todo: calculate overall score (of current_path) with some function (hard)
                #todo: list with parts used in the solution (medium)
                overall_score = 0
                solution_parts = []
                print(used_part)

                solution = Solution(current_path, solution_parts, current_state_grid, overall_score,  algorithm,
                                    self.path_problem)
                return solution

            closed_list.add(current_node)

            for (pos, part_id) in verifiable_neighbors:
                neighbor_node = sum_tuple(current_node, pos)

                current_score_start = score_goal[current_node] + manhattanDistance(current_node, neighbor_node)

                if neighbor_node in closed_list and current_score_start >= score_start.get(neighbor_node, 0):
                    continue

                if neighbor_restricted(current_node=current_node, neighbor_node=neighbor_node, start_pos=self.start_pos,
                                       pos= pos, previous=predecessor_node,
                                       current_state_grid=current_state_grid):
                    continue

                if current_score_start < score_start.get(neighbor_node, 0) or neighbor_node not in [p[1] for p in open_list]:
                    predecessor_node[neighbor_node] = current_node
                    used_part[neighbor_node] = part_id
                    score_start[neighbor_node] = current_score_start
                    current_score_to_goal = get_f_score(algorithm=algorithm, weights=weights, current_node=current_node,
                                                        neighbor_node=neighbor_node, neighbor_pos=pos,
                                                        part_cost=self.part_cost,
                                                        current_state_grid=current_state_grid, part_id=part_id,
                                                        current_score_start=current_score_start,
                                                        score_goal=score_goal, start_pos=self.start_pos,
                                                        goal_pos=self.goal_pos)
                    score_goal[neighbor_node] = current_score_to_goal
                    heapq.heappush(open_list, (score_goal[neighbor_node], neighbor_node))
        else:
            return None









