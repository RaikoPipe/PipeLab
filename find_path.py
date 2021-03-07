import numpy as np
import heapq
import numpy as np
from datetime import datetime
import LogicalVfunctions as lvf
import interpret_path as pint
from vpython import *
import object_classes
from copy import deepcopy
import time

priceList = [6.47,6.70,6.92,7.14,7.36]


def displayPlot_Call(x,y, start, goal, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType, pipeTypeDict, search_type):

    route, parts = displayPlot(tuple(map(lambda c,k: c-k, start, (1,1))),tuple(map(lambda c,k: c-k, goal, (1,1))),x,y, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType, pipeTypeDict, search_type)

    return route, parts

global showtime

def displayPlot(start_point,goal_point,x,y, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType, pipeTypeDict,search_type):

    grid = lvf.glG_Call(x, y)  # 0s are positions we can travel on, 1s are walls(obstacles or already placed pipes)
    if search_type == "dijkstra":
        route, parts = dijkstra(grid, start_point, goal_point, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType, 1, y, pipeTypeDict, False)
    elif search_type == "best-first":
        route, parts = bestFirstSearch(grid, start_point, goal_point, shiftpos, startAxis, goalAxis, testingPath, testedPath,
                             heuristicType, 1, y, pipeTypeDict, False)
    else:route, parts = astar(grid, start_point, goal_point, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType, 1, y, pipeTypeDict, False)



    if isinstance(route, str):
        print(route)
        return False, False
    else:
        route = route + [start_point]
        route = route[::-1]

        return route, parts


'''Custom Restriction Set:'''
def directional_rules_apply(current, current_neighbor, shiftpos):
    if current == (current[0], shiftpos - 1) and current_neighbor[1] > 0:
        return False

    elif current == (current[0], shiftpos +1) and current_neighbor[1] < 0:
        return False

    else:
        return True

def direction_is_restricted(cameFromDifference, currentNeighbor):
    if cameFromDifference[0] > 0 and currentNeighbor[0] != 0: #if true, then neighbor that wants to go horizontally is disallowed
        return True
    elif cameFromDifference[1] > 0 and currentNeighbor[1] != 0: #if true, then neighbor that wants to go vertically is disallowed
        return True
    else: return False

def direction_violation(diff, current, neighbor, c):
    if current == (current[0], c) and neighbor[1] > 0:
        return False
    elif current == (current[0], c +2) and neighbor[1] < 0:
        return False
    else:
        if diff[0] > 0 and neighbor[0] != 0:  # if true, then neighbor that wants to go horizontally is disallowed
            return True
        elif diff[1] > 0 and neighbor[1] != 0:  # if true, then neighbor that wants to go vertically is disallowed
            return True
        else:
            return False

def start_is_restricted(current_neighbor, startAxis):
    if startAxis == lvf.up and current_neighbor[1] > 0:
        return False
    elif startAxis == lvf.down and current_neighbor[1] < 0:
        return False
    elif startAxis == lvf.right and current_neighbor[0] > 0:
        return False
    elif startAxis == lvf.left and current_neighbor[0] < 0:
        return False
    else: return True

def dimension_shift_violation(current, neighbor, z):
    if neighbor[1] > z and current[1] < z:
        return True
    elif neighbor[1] < z+1 and current[1] > z+1:
        return True
    else:
        return False

def out_of_bounds(current, neighbor, n, array):


    if 0 <= neighbor[0] < array.shape[0]:

        if 0 <= neighbor[1] < array.shape[1]:
            diff = abs(n[0] - n[1])
            addX = 0
            addY = 0
            if n[0] > 0:
                addX = 1
            elif n[0] < 0:
                addX = -1
            elif n[1] > 0:
                addY = 1
            else:
                addY = -1
            for i in range(1, diff+1):
                pos = (current[0]+ addX*i, current[1] + addY*i)
                if array[pos] == 1:
                    return True
        else:
            return True # array bound y walls

    else:
        return True         # array bound x walls
    return False



def neighborChanger(standardNeighbors, changetuple, case):
    newNeighbors = {}
    for index, (tuple, type) in enumerate(standardNeighbors.items()):
        xCoord = tuple[0]
        yCoord = tuple[1]


        if case == "AddToPositiveOnly":
            if xCoord > 0:
                xCoord = xCoord + changetuple[0]
            elif yCoord > 0:
                yCoord = yCoord + changetuple[1]
        elif case == "AddToNegativeOnly":
            if xCoord < 0:
                xCoord = xCoord + changetuple[0]
            elif yCoord < 0:
                yCoord = yCoord + changetuple[1]
        else:
            if xCoord < 0:
                xCoord = xCoord + abs(changetuple[0])
            elif xCoord > 0:
                xCoord = xCoord + changetuple[0]

            if yCoord < 0:
                yCoord = yCoord + abs(changetuple[1])
            elif yCoord > 0:
                yCoord = yCoord + changetuple[1]


        newNeighbors[(xCoord, yCoord)] = type

    # if specialCase == True:
    #     specialNeighbors = []
    #     for count, tuple in enumerate(newNeighbors):
    #
    #         if tuple[0] >= topGoalDistance:
    #             continue
    #         elif tuple[1] >= topGoalDistance:
    #             continue
    #         else:
    #             specialNeighbors.append(tuple)
    #
    #     newNeighbors = specialNeighbors
    #
    #
    return newNeighbors

def buildRoute(current, came_from):
    data = []

    while current in came_from:
        data.append(current)
        current = came_from[current]

    return data

def buildParts(route, dict):
    parts = {}
    for idx, (x,y) in enumerate(route):
        #dont check next point if last point has been reached
        if idx == len(route)-1:
            break

        #set pointA and pointB, calculate difference
        current=(x,y)
        og_part = dict.get(current)
        parts[current] = og_part
    return

def stockCheck(path, type_dict, part_dict, unlimited_parts):
    if not unlimited_parts:
        availableParts = pint.pipe_stock_check(path, type_dict, part_dict)
        return availableParts

def getAxis(n):
    x=0
    y=0
    if n[0] != 0:
        x = (n[0])**0
    else:
        y = (n[1])**0
    if n[0]<0:
        x = -x
    elif n[1]<0:
        y = -y
    return (x,y)

def changeNeighbors(Neighbors, axis):
    changedNeighbors = {}
    for index, (n, t) in enumerate(Neighbors.items()):
        nAxis = getAxis(n)
        if nAxis == (axis.x,axis.y):
            changedNeighbors[(t*nAxis[0],t*nAxis[1])] = t
        else:
            changedNeighbors[n] = t
    return changedNeighbors

def zChangeNeighbors(Neighbors, add, axis):
    changedNeighbors = {}
    for index, (n, t) in enumerate(Neighbors.items()):
        nAxis = getAxis(n)
        if nAxis == (axis.x,axis.y):
            changedNeighbors[(n[0], n[1] + add)] = t
        else:
            changedNeighbors[n] = t
    return changedNeighbors


def positionDependence(Neighbors, start, startAxis, goal,goalAxis, current, z):
    newNeighbors = deepcopy(Neighbors)
    if current == start:
        newNeighbors = changeNeighbors(Neighbors, startAxis)
    else:
        for index, (n, t) in enumerate(Neighbors.items()):
            goalDistance = (np.abs(goal[0] - current[0]), np.abs(goal[1]-current[1]))
            absGoalDistance = np.abs(np.abs(goal[0] - current[0]) - np.abs(goal[1]-current[1]))
            if ((goalDistance[0] == t or goalDistance[1] == t) and absGoalDistance == t) or (goal == (current[0] + n[0], current[1] + n[1])
                    and (goalDistance[0] == n[0] or goalDistance[1] == n[1])):
                newNeighbors = changeNeighbors(Neighbors, -goalAxis)
                break
    if current == (current[0], z):
        add= 1
        newNeighbors = zChangeNeighbors(newNeighbors, add, lvf.up)
    elif current == (current[0],z+1):
        add= -1
        newNeighbors = zChangeNeighbors(newNeighbors, add, lvf.down)
    return newNeighbors

# fixme: There is a neighbor somewhere that shouldnt work with length 5
'Custom Neighbor Set'
def determineNeighbors(standardNeighbors, pipeTypeDict, current, start, goal, startAxis, goalAxis, shiftpos, yDots,
                       unlimited_parts, part_dict, route):


    neighbors = deepcopy(standardNeighbors)


    currToGoalDifference = (goal[0] - current[0], goal[1] - current[1])

    #current is at start
    if current == start:
        if startAxis == lvf.up:
            neighbors = neighborChanger(neighbors, (0,-1), "AddToPositiveOnly")
        elif startAxis == lvf.right:
            neighbors = neighborChanger(neighbors, (-1, 0), "AddToPositiveOnly")
        elif startAxis == lvf.left:
            neighbors = neighborChanger(neighbors, (1, 0), "AddToNegativeOnly")
        else:
            neighbors = neighborChanger(neighbors, (0, 1), "AddToNegativeOnly")

        return neighbors

    #current is not in shiftpos but goal is in reach
    # ->check if we can close the distance by using a pipe without a corner first
    #goal is one same horizontal axis

    values = neighbors.values()
    max_value = max(values) + 1

    if current[0] == goal[0] and 0 < currToGoalDifference[1] <= max_value and goalAxis == lvf.down:
        neighbors = neighborChanger(neighbors, (0, -1), "AddToPositiveOnly")
        return neighbors
    elif current[0] == goal[0] and 0 > currToGoalDifference[1] >= -max_value and goalAxis == lvf.up:

        neighbors = neighborChanger(neighbors, (0, 1), "AddToNegativeOnly")

        return neighbors
    #goal is on same vertical axis
    elif current[1] == goal[1] and 0 < currToGoalDifference[0] <= max_value and goalAxis == lvf.left:
        neighbors = neighborChanger(neighbors, (-1, 0), "AddToPositiveOnly")
        return neighbors
    elif current[1] == goal[1] and 0 > currToGoalDifference[0] >= -max_value and goalAxis == lvf.right:
        neighbors = neighborChanger(neighbors, (1, 0), "AddToNegativeOnly")
        return neighbors

    if current == (current[0], shiftpos - 1):
        #we are not on same horizontal axis as goal and at shiftpos-1
        if current == (current[0], shiftpos - 1) and current[0] == goal[0] and goalAxis == lvf.down:
            neighbors = neighborChanger(neighbors, (0, 1), "AddToPositiveOnly")
            return neighbors
        elif current == (current[0],shiftpos-1):
            neighbors = neighborChanger(neighbors, (0, 1), "AddToPositiveOnly")
            return neighbors

        return neighbors
    return neighbors


def heuristic(a,b):
    #manhattan distance
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    return distance
    # manhattan distance: np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    # np.sqrt((b[0] - a[0])** 2  + (b[1] - a[1]) ** 2)
    #return distance

def partHeuristic(length, current, neighbor, goal):
    cost = priceList[length-1]

    distance = length * cost

    neighToGoalDistance = np.abs(goal[0] - neighbor[0]) + np.abs(goal[1] - neighbor[1])  # manhattan distance from neigh to goal
    currToGoalDistance = np.abs(goal[0] - current[0]) + np.abs(goal[1] - current[1])  # manhattan distance from a to goal
    if currToGoalDistance > neighToGoalDistance:
        distance = length ** 2 * cost

    return distance



def modified_heuristic(a, b, currentNeighbor, heuristicType, weight, goal):
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])

    if heuristicType == "modified":
        neighToGoalDistance = np.abs(goal[0] - currentNeighbor[0]) + np.abs(goal[1] - currentNeighbor[1])  # manhattan distance from neigh to goal
        currToGoalDistance = np.abs(goal[0] - a[0]) + np.abs(goal[1] - a[1])  # manhattan distance from a to goal
        if currToGoalDistance < neighToGoalDistance:
            distance = np.abs(b[0] - a[0])**2 + np.abs(b[1] - a[1])**2

    return distance



# def optimizeRoute(grid, start_point, goal_point, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType, yDots):
#     routeList = []
#     bestRoute = []
#
#     currentRoute = astar(grid, start_point, goal_point, shiftpos, startAxis, goalAxis, testingPath, testedPath,
#                          heuristicType, 0, 0, yDots, False)
#     Objects.resetShowcase()
#
#     if not isinstance(currentRoute, list):
#         return "Creating route is not possible"
#
#
#     for i in range(0,5):
#         for j in range(-2,5):
#             Objects.resetShowcase()
#             gWeight = i/4
#             fWeight = j/4
#
#             currentRoute = astar(grid, start_point, goal_point, shiftpos, startAxis, goalAxis,testingPath,testedPath,
#                                  heuristicType, gWeight, fWeight, yDots, False)
#             if isinstance(currentRoute, str):
#                 continue
#
#             dotCost = 0
#             sumCost = 0
#             for idx, (x,y) in enumerate(currentRoute):
#
#                 #dont check next point if last point has been reached
#                 if idx == len(currentRoute)-1:
#                     break
#
#                 #set pointA and pointB, calculate difference
#                 pointA=currentRoute[idx]
#                 pointB=currentRoute[idx+1]
#                 differenceX = list(pointB)[0] - list(pointA)[0]
#                 differenceY = list(pointB)[1] - list(pointA)[1]
#
#                 # cost, dots = determineCostAndDots(differenceX, differenceY)
#                 # sumCost += cost
#                 # dotCost += dots
#                 #dotList.append(dots)
#             heapq.heappush(routeList, (sumCost, dotCost, currentRoute))
#     #print(heuristicType, routeList)
#     try:
#
#         chosenRoute = heapq.heappop(routeList)
#         #print(chosenRoute)
#         bestRoute = chosenRoute[2]
#         return bestRoute
#     except:
#         return "Creating route is not possible"


def get_standard_neighbors(dict):
    neighbors = {}
    for key, (type, count) in enumerate(dict.items()):
        neighbors[(type+1,0)] = type
        neighbors[(-type-1, 0)] = type
        neighbors[(0,type+1)] = type
        neighbors[(0,-type-1)] = type
    return neighbors

def dijkstra(array, start, goal, shiftpos, startAxis, goalAxis, testingPath,testedPath, heuristicType, weight,  yDots, pipeTypeDict, unlimited_parts):
    execTimeFailure = open("execTimeFailure.txt", "a")
    execTimeSuccess = open("execTimeSuccess.txt", "a")
    startTime = datetime.now()
    if array[start] == 1:
        return "Start point is blocked and therefore goal cant be reached", False
    elif array[goal] == 1:
        return "Goal point is blocked and therefore cant be reached", False

    if heuristicType == "intelligent":
        speed = 0.01
    else:
        speed = 0.1

    close_set = set()

    came_from = {}

    part_dict = {}

    gscore = {start: 0}

    oheap = []

    heapq.heappush(oheap, (gscore[start], start))
    count = 0
    neiCount = 0


    standardNeighbors = get_standard_neighbors(pipeTypeDict)


    while oheap:
        current = heapq.heappop(oheap)[1]

        if testingPath == True:
            if count > 0:
                currentBox.obj.color = vector(0,0.5,0)
                currentBox.obj.opacity = 0.5
                currentBox=object_classes.currentDebugBox((current[0] + 1, current[1] + 1))
            else:
                currentBox=object_classes.currentDebugBox((current[0] + 1, current[1] + 1))
                count +=1

        current_route = buildRoute(current, came_from)
        current_route = current_route + [start]
        current_route = current_route[::-1]

        L = stockCheck(current_route, pipeTypeDict, part_dict, unlimited_parts)
        N = pint.set_standard_neighbors(L)
        Neighbors = positionDependence(N, start, startAxis, goal, goalAxis, current, shiftpos-1)

        if pint.pathCheck(current_route, array):
            continue

        close_set.add(current) #add the from oheap popped coordinate to the closed list

        if current == goal:
            if testingPath == True:
                if neiCount > 0:
                    neighBox.obj.visible = False
                    neighBox.obj.delete()
                    neiCount = 0
            if testingPath == True:
                if count > 0:
                    currentBox.obj.color = vector(0, 0.5, 0)
                    currentBox.obj.opacity = 0.5
            data = buildRoute(current, came_from)
            exectime = datetime.now() - startTime
            execTimeSuccess.write(str(exectime.total_seconds())+"\n")
            if unlimited_parts:
                print("Optimal route is possible with more parts")
            else:
                return data, part_dict

        close_set.add(current) #add the from oheap popped coordinate to the closed list

        if testingPath == True:
            if neiCount > 0:
                neighBox.obj.visible = False
                neighBox.obj.delete()
                neiCount = 0
        for i, j in Neighbors:
            current_neighbor = (i, j)
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            # if heuristicType == "price-based":
            #     tentative_g_score = gscore[current] + partHeuristic(nextNeighbors.get((i, j)), current,
            #                                                          neighbor, goal)
            # else:
            #     tentative_g_score = gscore[current] + modified_heuristic(current, neighbor,
            #                                                               neighbor, heuristicType, weight,
            #                                                               goal)
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            #restriction set
            if dimension_shift_violation(current, neighbor, shiftpos-1):
                continue
            if current != start:
                came_fromDifference = [abs(current[0] - came_from[current][0]), abs(current[1] - came_from[current][1])]
                if direction_violation(came_fromDifference, current, current_neighbor, shiftpos-1):
                    continue
            if out_of_bounds(current, neighbor, current_neighbor, array):
                continue
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                if testingPath == True:
                    if neiCount > 0:
                        neighBox.obj.visible = False
                        neighBox.obj.delete()
                        neighBox = object_classes.neighborDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                        time.sleep(speed)
                    else:
                        neighBox = object_classes.neighborDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                        neiCount += 1
                        time.sleep(speed)
                if testedPath == True:
                    testedBox = object_classes.possiblePositionDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                came_from[neighbor] = current
                part_dict[neighbor] = Neighbors.get((i,j))
                gscore[neighbor] = tentative_g_score

                # if heuristicType == "price-based":
                #     fscore[neighbor] = tentative_g_score + partHeuristic(nextNeighbors.get((i,j)),current, neighbor, goal)
                # else:
                #     fscore[neighbor] = tentative_g_score + modified_heuristic(neighbor, goal, current, heuristicType, weight, goal)
                #fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (gscore[neighbor], neighbor))
    # if not unlimited_parts:
    #     exectime = datetime.now() - startTime
    #     execTimeFailure.write(str(exectime.total_seconds()) + "\n")
    #     print("Route creation is not possible with limited parts")
    #     astar(array, start, goal, shiftpos, startAxis, goalAxis, testingPath,testedPath, heuristicType, gWeight, fWeight, yDots, pipeTypeDict, True)
    print("Creating route is not possible, even with unlimited parts")
    exectime = datetime.now() - startTime
    execTimeFailure.write(str(exectime.total_seconds()) + "\n")
    return "no route found", False

def astar(array, start, goal, shiftpos, startAxis, goalAxis, testingPath,testedPath, heuristicType, weight,  yDots, pipeTypeDict, unlimited_parts):
    execTimeFailure = open("execTimeFailure.txt", "a")
    execTimeSuccess = open("execTimeSuccess.txt", "a")
    startTime = datetime.now()
    if array[start] == 1:
        return "Start point is blocked and therefore goal cant be reached", False
    elif array[goal] == 1:
        return "Goal point is blocked and therefore cant be reached", False

    if heuristicType == "intelligent":
        speed = 0.01
    else:
        speed = 0.1

    close_set = set()

    came_from = {}

    part_dict = {}

    gscore = {start: 0}

    fscore = {start: heuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
    count = 0
    neiCount = 0


    standardNeighbors = get_standard_neighbors(pipeTypeDict)


    while oheap:
        current = heapq.heappop(oheap)[1]

        if testingPath == True:
            if count > 0:
                currentBox.obj.color = vector(0,0.5,0)
                currentBox.obj.opacity = 0.5
                currentBox=object_classes.currentDebugBox((current[0] + 1, current[1] + 1))
            else:
                currentBox=object_classes.currentDebugBox((current[0] + 1, current[1] + 1))
                count +=1

        current_route = buildRoute(current, came_from)
        current_route = current_route + [start]
        current_route = current_route[::-1]

        L = stockCheck(current_route, pipeTypeDict, part_dict, unlimited_parts)
        N = pint.set_standard_neighbors(L)
        Neighbors = positionDependence(N, start, startAxis, goal, goalAxis, current, shiftpos-1)

        if pint.pathCheck(current_route, array):
            continue

        close_set.add(current) #add the from oheap popped coordinate to the closed list
        if current == goal:
            if testingPath == True:
                if neiCount > 0:
                    neighBox.obj.visible = False
                    neighBox.obj.delete()
                    neiCount = 0
            if testingPath == True:
                if count > 0:
                    currentBox.obj.color = vector(0, 0.5, 0)
                    currentBox.obj.opacity = 0.5
            if unlimited_parts:
                #TODO: check how many parts are needed to complete optimal route"
                print("Route Creation is possible with more parts")
            data = buildRoute(current, came_from)
            exectime = datetime.now() - startTime
            execTimeSuccess.write(str(exectime.total_seconds())+"\n")
            if unlimited_parts:
                print("Optimal route is possible with more parts")
            else:
                return data, part_dict

        close_set.add(current) #add the from oheap popped coordinate to the closed list

        if testingPath == True:
            if neiCount > 0:
                neighBox.obj.visible = False
                neighBox.obj.delete()
                neiCount = 0
        for i, j in Neighbors:
            current_neighbor = (i, j)
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            # if heuristicType == "price-based":
            #     tentative_g_score = gscore[current] + partHeuristic(nextNeighbors.get((i, j)), current,
            #                                                          neighbor, goal)
            # else:
            #     tentative_g_score = gscore[current] + modified_heuristic(current, neighbor,
            #                                                               neighbor, heuristicType, weight,
            #                                                               goal)
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            #restriction set
            if dimension_shift_violation(current, neighbor, shiftpos-1):
                continue
            if current != start:
                came_fromDifference = [abs(current[0] - came_from[current][0]), abs(current[1] - came_from[current][1])]
                if direction_violation(came_fromDifference, current, current_neighbor, shiftpos-1):
                    continue
            if out_of_bounds(current, neighbor, current_neighbor, array):
                continue
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                if testingPath == True:
                    if neiCount > 0:
                        neighBox.obj.visible = False
                        neighBox.obj.delete()
                        neighBox = object_classes.neighborDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                        time.sleep(speed)
                    else:
                        neighBox = object_classes.neighborDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                        neiCount += 1
                        time.sleep(speed)
                if testedPath == True:
                    testedBox = object_classes.possiblePositionDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                came_from[neighbor] = current
                part_dict[neighbor] = Neighbors.get((i,j))
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                # if heuristicType == "price-based":
                #     fscore[neighbor] = tentative_g_score + partHeuristic(nextNeighbors.get((i,j)),current, neighbor, goal)
                # else:
                #     fscore[neighbor] = tentative_g_score + modified_heuristic(neighbor, goal, current, heuristicType, weight, goal)
                #fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # if not unlimited_parts:
    #     exectime = datetime.now() - startTime
    #     execTimeFailure.write(str(exectime.total_seconds()) + "\n")
    #     print("Route creation is not possible with limited parts")
    #     astar(array, start, goal, shiftpos, startAxis, goalAxis, testingPath,testedPath, heuristicType, gWeight, fWeight, yDots, pipeTypeDict, True)
    print("Creating route is not possible, even with unlimited parts")
    exectime = datetime.now() - startTime
    execTimeFailure.write(str(exectime.total_seconds()) + "\n")
    return "no route found", False

def bestFirstSearch(array, start, goal, shiftpos, startAxis, goalAxis, testingPath,testedPath, heuristicType, weight,  yDots, pipeTypeDict, unlimited_parts):
    execTimeFailure = open("execTimeFailure.txt", "a")
    execTimeSuccess = open("execTimeSuccess.txt", "a")
    startTime = datetime.now()
    if array[start] == 1:
        return "Start point is blocked and therefore goal cant be reached", False
    elif array[goal] == 1:
        return "Goal point is blocked and therefore cant be reached", False

    if heuristicType == "intelligent":
        speed = 0.01
    else:
        speed = 0.1

    close_set = set()

    came_from = {}

    part_dict = {}

    hscore = {start: heuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (hscore[start], start))
    count = 0
    neiCount = 0

    while oheap:
        current = heapq.heappop(oheap)[1]

        if testingPath == True:
            if count > 0:
                currentBox.obj.color = vector(0,0.5,0)
                currentBox.obj.opacity = 0.5
                currentBox=object_classes.currentDebugBox((current[0] + 1, current[1] + 1))
            else:
                currentBox=object_classes.currentDebugBox((current[0] + 1, current[1] + 1))
                count +=1

        current_route = buildRoute(current, came_from)
        current_route = current_route + [start]
        current_route = current_route[::-1]

        L = stockCheck(current_route, pipeTypeDict, part_dict, unlimited_parts)
        N = pint.set_standard_neighbors(L)
        Neighbors = positionDependence(N, start, startAxis, goal, goalAxis, current, shiftpos-1)

        if pint.pathCheck(current_route, array):
            continue

        close_set.add(current) #add the from oheap popped coordinate to the closed list
        if current == goal:
            if testingPath == True:
                if neiCount > 0:
                    neighBox.obj.visible = False
                    neighBox.obj.delete()
                    neiCount = 0
            if testingPath == True:
                if count > 0:
                    currentBox.obj.color = vector(0, 0.5, 0)
                    currentBox.obj.opacity = 0.5
            if unlimited_parts:
                #TODO: check how many parts are needed to complete optimal route"
                print("Route Creation is possible with more parts")
            data = buildRoute(current, came_from)
            exectime = datetime.now() - startTime
            execTimeSuccess.write(str(exectime.total_seconds())+"\n")
            if unlimited_parts:
                print("Optimal route is possible with more parts")
            else:
                return data, part_dict

        close_set.add(current) #add the from oheap popped coordinate to the closed list

        if testingPath == True:
            if neiCount > 0:
                neighBox.obj.visible = False
                neighBox.obj.delete()
                neiCount = 0
        for i, j in Neighbors:
            current_neighbor = (i, j)
            neighbor = current[0] + i, current[1] + j
            tentative_h_score = hscore[current] + heuristic(current, neighbor)

            if neighbor in close_set:
                continue
            #restriction set
            if dimension_shift_violation(current, neighbor, shiftpos-1):
                continue
            if current != start:
                came_fromDifference = [abs(current[0] - came_from[current][0]), abs(current[1] - came_from[current][1])]
                if direction_violation(came_fromDifference, current, current_neighbor, shiftpos-1):
                    continue
            if out_of_bounds(current, neighbor, current_neighbor, array):
                continue
            if tentative_h_score < hscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                if testingPath == True:
                    if neiCount > 0:
                        neighBox.obj.visible = False
                        neighBox.obj.delete()
                        neighBox = object_classes.neighborDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                        time.sleep(speed)
                    else:
                        neighBox = object_classes.neighborDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                        neiCount += 1
                        time.sleep(speed)
                if testedPath == True:
                    testedBox = object_classes.possiblePositionDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                came_from[neighbor] = current
                part_dict[neighbor] = Neighbors.get((i,j))
                hscore[neighbor] = tentative_h_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (hscore[neighbor], neighbor))
    # if not unlimited_parts:
    #     exectime = datetime.now() - startTime
    #     execTimeFailure.write(str(exectime.total_seconds()) + "\n")
    #     print("Route creation is not possible with limited parts")
    #     astar(array, start, goal, shiftpos, startAxis, goalAxis, testingPath,testedPath, heuristicType, gWeight, fWeight, yDots, pipeTypeDict, True)
    print("Creating route is not possible, even with unlimited parts")
    exectime = datetime.now() - startTime
    execTimeFailure.write(str(exectime.total_seconds()) + "\n")
    return "no route found", False