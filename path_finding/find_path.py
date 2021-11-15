import heapq
import numpy as np
from datetime import datetime
from path_finding import interpret_path as pint
from vpython import *

from path_finding.path_data_classes import Solution, PathProblem
from rendering import object_classes, positional_functions_old as lvf
from copy import deepcopy
import time

#todo: full refactoring
#save showcase inside a show_case_grid

# prepare path finding, returns numpy matrix (which is rotated by 90°) with obstacle pos (1) and planned layout pos (2)
# def displayPlot_Call(x,y, s, t, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType, pipeTypeDict, search_type,gC,gP,gMinO):
#     s = (s[0]-1,s[1]-1)
#     t = (t[0] - 1, t[1] - 1)
#     grid = lvf.glG_Call(x, y)  #numpy matrix with obstacle positions #altM is Matrix with obstacles and planned layout
#     if search_type == "dijkstra":
#         route, parts, altM = dijkstra(grid, s, t, shiftpos, startAxis, goalAxis, testingPath, testedPath, heuristicType, 1, y, pipeTypeDict, False)
#     elif search_type == "best-first":
#         route, parts, altM = bestFirstSearch(grid, s, t, shiftpos, startAxis, goalAxis, testingPath, testedPath,
#                                        heuristicType, 1, y, pipeTypeDict, False)
#     elif search_type == "multicriteria astar":
#         route, parts, altM = multicriteriaAstar(grid, s, t, shiftpos, startAxis, goalAxis, testingPath, testedPath,
#                                           heuristicType, 1, y, pipeTypeDict, False, gC, gP, gMinO)
#     else:
#         route, parts, altM = astar(grid, s, t, shiftpos, startAxis, goalAxis, testingPath, testedPath, heuristicType, 1, y, pipeTypeDict, False)
#
#     if isinstance(route, str):
#         print(route)
#         return False, False
#     else:
#         route = route + [s]
#         route = route[::-1]
#         altM[s] = 2
#         return route, parts
#
# global showtime


#restriction functions start here

#checks if direction is the same as previous (which is not allowed)
def directionRestricted(diff, v, n, z):
    if v[1] == z and n[1] > 0:
        return False
    elif v[1] == z + 2 and n[1] < 0:
        return False
    else:
        if diff[0] > 0 and n[0] != 0:  # if true, then neighbor that wants to go horizontally is disallowed
            return True
        elif diff[1] > 0 and n[1] != 0:  # if true, then neighbor that wants to go vertically is disallowed
            return True
        else:
            return False


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
    # if current == (current[0], z):
    #     add= 1
    #     newNeighbors = ChangeZNeighbors(newNeighbors, add, lvf.up)
    # elif current == (current[0],z+2):
    #     add= -1
    #     newNeighbors = ChangeZNeighbors(newNeighbors, add, lvf.down)
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

#additional cost functions start here

#redundant way of getting prices, but it will do for now
# priceList = [1.15,1.38,1.60,1.82,2.04]
# priceListWithCorner = [6.47,6.70,6.92,7.14,7.36]
priceList = [1.15,1.38,1.60,20,2.04]
priceListWithCorner = [6.47,6.70,6.92,25.32,7.36]
upperBoundPL = 0
upperBoundPLWC = 0

for i in range(len(priceList)):
    valuePL = priceList[i]/(i+1)
    valuePLWC = priceListWithCorner[i]/(i+1)
    if valuePL > upperBoundPL:
        upperBoundPL = valuePL
    if valuePLWC > upperBoundPLWC:
        upperBoundPLWC = valuePLWC

def costP(n, part):
    nLength = abs(n[0]-n[1])
    if nLength == part:
        upperBound = upperBoundPL
        P = priceList[part-1]/nLength
    else:
        upperBound = upperBoundPLWC
        P = priceListWithCorner[part - 1]/nLength
    return P/upperBound

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
    if current_node != start_pos:
        previous_pos = (abs(current_node[0]-previous[current_node][0]),abs(current_node[1]-previous[current_node][1]))
        if directionRestricted(previous_pos, current_node, pos):
            return True

    if not outOfBounds(neighbor_node, current_state_grid):
        if collidedObstacle(current_node, pos, current_state_grid):
            return True
    else: return True

    return False

def get_f_score(algorithm, weights, current_node, neighbor_node, verifiable_neighbors_pos, pos, current_state_grid, current_score_to_start,
                score_to_goal, start_pos, goal_pos):
    score = 0
    if algorithm == "mca*":
        distance = (current_score_to_start +
                    manhattanDistance(neighbor_node, goal_pos)) / score_to_goal[start_pos] * weights.path_length
        cost = costP(pos, verifiable_neighbors_pos.get(pos)) * weights.cost
        distance_to_obstacles = costMinO(current_state_grid, current_node, pos)* weights.distance_to_obstacles
        score = distance + cost + distance_to_obstacles
    return score


#todo: crossingRestricted not needed anymore
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
    #todo: for partial solution finding, we need to separate pipes and corners.
    # i.e. there needs to be a list that point to the previously used part -> used_parts (corner can be identified as 0)
    def find_path(self, weights, algorithm):
        closed_list = set()
        open_list = []

        predecessor_node = {}

        score_start = {self.start_pos:0} # G
        score_goal = {self.start_pos: manhattanDistance(self.start_pos,self.goal_pos)} # F

        used_parts = {}

        heapq.heappush(open_list, (score_goal[self.start_pos], self.start_pos))
        while open_list:
            current_node = heapq.heappop(open_list)[1] # pops the node with the smallest score from open_list
            current_path = buildPath(current_node, predecessor_node, self.start_pos)
            available_parts = pint.pipe_stock_check(current_path, self.pipe_stock, used_parts)
            # verifiable_neighbors =
            # todo: determine next move choices, set neighbors accordingly
            if current_node != self.start_pos:
                if used_parts[predecessor_node[current_node]] == 0:
                    # predecessor is corner
                    verifiable_neighbors = pint.get_neighbors({1:0})
                else:
                    # predecessor is pipe
                    verifiable_neighbors = positionDependence(pint.get_neighbors([0]), self.start_pos,
                                                              self.start_axis, self.goal_pos, self.goal_axis,
                                                              current_node)
            else:
                # current node is start
                verifiable_neighbors = positionDependence(pint.get_neighbors(available_parts), self.start_pos,
                                                          self.start_axis, self.goal_pos, self.goal_axis, current_node)

            current_state_grid = pint.getAlteredMatrix(current_path, self.state_grid)

            if current_node == self.goal_pos:
                # search is finished!
                overall_score = 0 # todo: calculate overall score with function
                #todo: list with parts used in the solution
                solution = Solution(current_path, used_parts, current_state_grid, overall_score,  algorithm,
                                    self.path_problem)
                return solution

            closed_list.add(current_node)



            for pos in verifiable_neighbors:
                neighbor_node = (current_node[0]+ pos[0], current_node[1] + pos[1])

                current_score_start = score_goal[current_node] + manhattanDistance(current_node, neighbor_node)

                if neighbor_node in closed_list and current_score_start >= score_start.get(neighbor_node, 0):
                    continue

                if neighbor_restricted(current_node=current_node, neighbor_node=neighbor_node, start_pos=self.start_pos,
                                       pos= pos, previous=predecessor_node,
                                       current_state_grid=current_state_grid):
                    continue

                if current_score_start < score_start.get(neighbor_node, 0) or neighbor_node not in [p[1] for p in open_list]:
                    predecessor_node[neighbor_node] = current_node
                    used_parts[verifiable_neighbors] = verifiable_neighbors.get(pos)
                    score_start[neighbor_node] = current_score_start
                    current_score_to_goal = get_f_score(algorithm, weights, current_node, neighbor_node,
                                                        verifiable_neighbors, pos, current_state_grid,
                                                        current_score_start, score_goal, self.start_pos,
                                                        self.goal_pos)
                    score_goal[neighbor_node] = current_score_to_goal
                    heapq.heappush(open_list, (score_goal[neighbor_node], neighbor_node))
            else:
                return None









