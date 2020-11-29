import numpy as np
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import LogicalVfunctions as lvf
from vpython import *
import Objects
import time


def displayPlot_Call(x,y, start, goal, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType):
    route_call = displayPlot(tuple(map(lambda c,k: c-k, start, (1,1))),tuple(map(lambda c,k: c-k, goal, (1,1))),x,y, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType)
    return route_call

global showtime

def displayPlot(start_point,goal_point,x,y, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType):


    grid = lvf.glG_Call(x, y)  # 0s are positions we can travel on, 1s are walls(obstacles or already placed pipes)
    if heuristicType == "intelligent":
        route = optimizeRoute(grid, start_point, goal_point, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType)
    else:
        weight = 0
        route = astar(grid, start_point, goal_point, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType, weight)
    if isinstance(route, str):
        print(route)
    else:
        route = route + [start_point]
        route = route[::-1]
        print("astar path creation successful")

        # plot map and path
        x_coords = []
        # y_coords = []
        # for i in (range(0,len(route))):
        #
        #     x = route[i][0]
        #
        #     y = route[i][1]
        #
        #     x_coords.append(x)
        #     y_coords.append(y)
        #
        # fig, ax = plt.subplots(figsize=(x, y))
        #
        # ax.imshow(grid, cmap=plt.cm.Dark2)
        #
        # ax.scatter(start_point[1], start_point[0], marker="*", color="green", s=200)
        #
        # ax.scatter(goal_point[1], goal_point[0], marker="*", color="orange", s=200)
        #
        # ax.plot(y_coords, x_coords, color="black")
        #
        # #plt.show()
        return route


'Custom Restriction Set:'
#check through the whole x-space the neighbor wants to occupy and check if it crosses an obstacle
def previousXoccupied(neighbor, current, array):

    difference = current[0]- neighbor[0]
    if difference > 0:
        for ix in range(difference-1):
            if array[neighbor[0] + (ix + 1), neighbor[1]] == 1:
                return True
        return False
    else:
        difference = abs(difference)
        for ix in range(difference-1):
            if array[neighbor[0] - (ix + 1), neighbor[1]] == 1:
                return True
        return False

#check through the whole y-space the neighbor wants to occupy and check if it crosses an obstacle
def previousYoccupied(neighbor, current, array):
    difference = current[1] - neighbor[1]
    if difference > 0:
        for ix in range(difference-1):
            if array[neighbor[0], neighbor[1] + (ix + 1)] == 1:
                return True
        return False
    else:
        difference = abs(difference)
        for ix in range(difference-1):
            if array[neighbor[0], neighbor[1] - (ix + 1)] == 1:
                return True

    return False

def directionalRulesApply(current, current_neighbor, shiftpos):
    if current == (current[0], shiftpos - 1) and current_neighbor[1] > 0:
        #we are at shiftpos-1 and want to go up
        return False

    elif current == (current[0], shiftpos +1) and current_neighbor[1] < 0:
        #we are at shiftpos+1 and want to go down
        return False

    else:
        return True

#check if restriction that it needs to change direction by 90° after laying a pipe or pipe combo isn't violated.
def isDirectionRestricted(cameFromDifference, currentNeighbor):
    if cameFromDifference[0] > 0 and currentNeighbor[0] != 0: #if true, then neighbor that wants to go horizontally is disallowed
        return True
    elif cameFromDifference[1] > 0 and currentNeighbor[1] != 0: #if true, then neighbor that wants to go vertically is disallowed
        return True
    else: return False

def startRestricted(current_neighbor, startAxis):
    if startAxis == lvf.up and current_neighbor[1] > 0:
        return False
    elif startAxis == lvf.down and current_neighbor[1] < 0:
        return False
    elif startAxis == lvf.right and current_neighbor[0] > 0:
        return False
    elif startAxis == lvf.left and current_neighbor[0] < 0:
        return False
    else: return True

def dimensionShiftViolated(j, shiftpos, current):
    if j > (shiftpos - 1) - current[1] and current[1] < (shiftpos - 1):
        return True

# fixme: There is a neighbor somewhere that shouldnt work with length 5
'Custom Neighbor Set'
def determineNeighbors(current, start, goal, goalAxis, shiftpos):
    currToGoalDifference = (goal[0] - current[0], goal[1] - current[1])

    #current is at start
    if current == start:
        # neighbor needs to be shortened by 1 for some reason if current = start
        neighbors = [(0, 7), (0, -7), (7, 0), (-7, 0), (0, 6), (0, -6), (6, 0), (-6, 0), (0, 5), (0, -5), (5, 0),
                     (-5, 0),
                     (0, 3), (0, -3), (3, 0), (-3, 0), (0, 2), (0, -2), (2, 0), (-2, 0), (0, 1), (0, -1), (1, 0),
                     (-1, 0)]
        return neighbors

    #current is not in shiftpos but goal is in reach
    # ->check if we can close the distance by using a pipe without a corner first
    #goal is one same horizontal axis
    if current[0] == goal[0] and currToGoalDifference[1] > 0 and currToGoalDifference[1] <= 8 and goalAxis == lvf.down:
        neighbors = [(0, 7), (0, 6), (0, 5), (0, 3), (0, 2), (0, 1), (0, -8), (8, 0), (-8, 0), (0, -7), (7, 0),
                     (-7, 0), (0, -6), (6, 0), (-6, 0), (0, -4), (4, 0), (-4, 0), (0, -3), (3, 0), (-3, 0), (0, -2),
                     (2, 0),(-2, 0)]
        return neighbors
    elif current[0] == goal[0] and currToGoalDifference[1] < 0 and currToGoalDifference[1] >= -8 and goalAxis == lvf.up:
        neighbors = [(0, -7), (0, -6), (0, -5), (0, -3), (0, -2), (0, -1),(0, 8),  (8, 0), (-8, 0),(0, 7),  (7, 0),
                     (-7, 0),(0, 6), (6, 0), (-6, 0),(0, 4), (4, 0), (-4, 0),(0, 3), (0, -3), (3, 0), (0, 2),
                     (2, 0),(-2, 0)]
        return neighbors
    #goal is on same vertical axis
    elif current[1] == goal[1] and currToGoalDifference[0] > 0 and currToGoalDifference[0] <= 8 and goalAxis == lvf.left:
        neighbors = [(7, 0), (6, 0), (5, 0), (3, 0), (2, 0), (1, 0), (0, 8), (0, -8), (-8, 0),(0, 7), (0, -7), (-7, 0),
                     (0, 6), (0, -6), (-6, 0), (0, 4), (0, -4),  (-4, 0),(0, 3), (0, -3),  (-3, 0),(0, 2), (0, -2),
                     (-2, 0)]
        return neighbors
    elif current[1] == goal[1] and currToGoalDifference[0] < 0 and currToGoalDifference[0] >= -8 and goalAxis == lvf.right:
        neighbors = [(-7, 0), (-6, 0), (-5, 0), (-3, 0), (-2, 0), (-1, 0), (0, 8), (0, -8), (8, 0), (0, 7), (0, -7),
                     (7, 0), (0, 6), (0, -6), (6, 0), (0, 4), (0, -4), (4, 0),(0, 3), (0, -3), (3, 0),(0, 2), (0, -2),
                     (2, 0)]
        return neighbors

    if current == (current[0], shiftpos - 1):
        #we are not on same horizontal axis as goal and at shiftpos-1
        if current == (current[0], shiftpos - 1) and current[0] == goal[0] and goalAxis == lvf.down:
            neighbors = [(0, -8), (8, 0), (-8, 0), (0, 8), (0, -7), (7, 0), (-7, 0), (0, 7), (0, -6), (6, 0),
                         (-6, 0),
                         (0, 5), (0, -4), (4, 0), (-4, 0), (0, 4), (0, -3), (3, 0), (-3, 0), (0, 3), (0, -2), (2, 0),
                         (-2, 0)]
            return neighbors
        elif current == (current[0],shiftpos-1):
            neighbors = [(0, 9), (0, -8), (8, 0), (-8, 0),(0, 8), (0, -7), (7, 0), (-7, 0),(0, 7), (0, -6), (6, 0), (-6, 0),
                        (0, 5), (0, -4), (4, 0), (-4, 0),(0, 4), (0, -3), (3, 0), (-3, 0),(0, 3), (0, -2), (2, 0),
                        (-2, 0)]
            return neighbors
        # we are not on same horizontal axis as goal and at shiftpos+1
        # elif current == (current[0],shiftpos+1): # we are not on same horizontal axis as goal
        #     neighbors = [(0, 8), (0, -9), (8, 0), (-8, 0),(0, 7), (0, -8), (7, 0), (-7, 0),(0, 6), (0, -7), (6, 0), (-6, 0),
        #                 (0, 4), (0, -5), (4, 0), (-4, 0),(0, 3), (0, -4), (3, 0), (-3, 0),(0, 2), (0, -3), (2, 0),
        #                 (-2, 0)]
        #     return neighbors
        # elif current == (current[0],shiftpos-1):
        #     neighbors = [(0, 9), (0, -8), (8, 0), (-8, 0),(0, 8), (0, -7), (7, 0), (-7, 0),(0, 7), (0, -6), (6, 0), (-6, 0),
        #                 (0, 5), (0, -4), (4, 0), (-4, 0),(0, 4), (0, -3), (3, 0), (-3, 0),(0, 3), (0, -2), (2, 0),
        #                 (-2, 0)]
        #     return neighbors
        # elif current == (current[0],shiftpos+1):
        #     neighbors = [(0, 8), (0, -9), (8, 0), (-8, 0),(0, 7), (0, -8), (7, 0), (-7, 0),(0, 6), (0, -7), (6, 0), (-6, 0),
        #                 (0, 4), (0, -5), (4, 0), (-4, 0),(0, 3), (0, -4), (3, 0), (-3, 0),(0, 2), (0, -3), (2, 0),
        #                 (-2, 0)]
    else:
        neighbors = [(0, 8), (0, -8), (8, 0), (-8, 0), (0, 7), (0, -7), (7, 0), (-7, 0), (0, 6), (0, -6), (6, 0),
                     (-6, 0),
                     (0, 4), (0, -4), (4, 0), (-4, 0), (0, 3), (0, -3), (3, 0), (-3, 0), (0, 2), (0, -2), (2, 0),
                     (-2, 0)]
        return neighbors


    # current is in shiftpos-1 or shiftpos+1
    #fixme: differentiate between close to goal or not
    #goal is not in reach



        #The following might be useless and wrong, since if the goal is in reach but still blocked it will be stuck
        # elif abs(currToGoalDifference[1]) <= 8 and current[0] == goal[0]: # we are on the same axis as goal and its reachable
        #     #goal is on same horizontal axis, goal is above current
        #     if current[0] == goal[0] and currToGoalDifference[1] > 0 and currToGoalDifference[1] <= 8 and goalAxis == lvf.down:
        #         neighbors = [(0, 8), (0, 7), (0, 6), (0, 4), (0, 3), (0, 2)]
        #         return neighbors
        #     #goal is on same horizontal axis, goal is below current
        #     elif current[0] == goal[0] and currToGoalDifference[1] < 0 and currToGoalDifference[1] >= -8 and goalAxis == lvf.up:
        #         neighbors = [(0, -8), (0, -7), (0, -6), (0, -4), (0, -3), (0, -2)]
        #         return neighbors
        # elif current[0] == goal[0] and abs(currToGoalDifference[1]) > 8: #we are on the same axis as goal, but its not reachable
        #     neighbors = [(8, 0), (-8, 0), (0, 8), (0, -8), (7, 0), (-7, 0), (0, 7), (0, -7), (6, 0),
        #                  (-6, 0),
        #                  (0, 5), (0, -5), (4, 0), (-4, 0), (0, 4), (0, -4), (3, 0), (-3, 0), (0, 3), (0, -3), (2, 0),
        #                  (-2, 0)]
        #     return neighbors



def determineTypeCost(currentNeighbor):
    x = abs(currentNeighbor[0])
    y = abs(currentNeighbor[1])
    if x == 9 or y == 9:
        costPerDot = 1.16
    elif x == 8 or y == 8:
        costPerDot = 1.28
    elif x == 7 or y == 7:
        costPerDot = 1.43
    elif x == 6 or y == 6:
        costPerDot = 1.55
    elif x == 5 or y == 5:
        costPerDot = 1.64
    elif x == 4 or y == 4:
        costPerDot = 1.73
    elif x == 3 or y == 3:
        costPerDot = 2.23
    elif x == 2 or y == 2:
        costPerDot = 3.24
    elif x == 1 or y == 1:
        costPerDot = 3.24
    else:
        type = "error"
        print("type doesnt exist")

    return costPerDot



def heuristic(a,b): #fixme: Artificially change the score to favor long pipes
    #manhattan distance
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    return distance
    # manhattan distance: np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    # np.sqrt((b[0] - a[0])** 2  + (b[1] - a[1]) ** 2)
    #return distance

def artificialHeuristic(a,b, goal, currentNeighbor, heuristicType, weight):
    neighToGoalDistance = np.abs(goal[0] - b[0]) + np.abs(goal[1] - b[1]) # manhattan distance from neigh to goal
    currToGoalDistance = np.abs(goal[0] - a[0]) + np.abs(goal[1] - a[1]) # manhattan distance from a to goal
    #if the neighbor decreases the distance to goal, reward algo
    distance = np.abs(goal[0] - b[0]) + np.abs(goal[1] - b[1])

    if heuristicType == "add":
        add = abs(currentNeighbor[0]/2+currentNeighbor[1]/2)
    elif heuristicType == "subtract":
        add = -abs(currentNeighbor[0]/2+currentNeighbor[1]/2)
    elif heuristicType == "intelligent":
        add = determineTypeCost(currentNeighbor)
    else:
        add = 0

    if heuristicType != "intelligent":
        if neighToGoalDistance < currToGoalDistance:
            artificialDistance = distance + add
        else:
            artificialDistance = distance

        return artificialDistance
    else:
        artificialDistance = distance + weight * add
        return artificialDistance

def determineCostAndDots(x,y):
    x = abs(x)
    y = abs(y)
    if x == 9 or y == 9:
        cost = 10.44
    elif x == 8 or y == 8:
        cost = 10.22
    elif x == 7 or y == 7:
        cost = 10.00
    elif x == 6 or y == 6:
        cost = 9.27
    elif x == 5 or y == 5:
        cost = 8.09
    elif x == 4 or y == 4:
        cost = 6.92
    elif x == 3 or y == 3:
        cost = 6.70
    elif x == 2 or y == 2:
        cost = 6.47
    elif x == 1 or y == 1:
        cost = 1.15
    else:
        type = "error"
        print("type doesnt exist")

    if x !=0:
        dots = x
    elif y != 0:
        dots = y

    return cost, dots



def optimizeRoute(grid, start_point, goal_point, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType):
    routeList = []

    for i in range(10):
        currentRoute = astar(grid, start_point, goal_point, shiftpos, startAxis, goalAxis,testingPath,testedPath, heuristicType, i)
        if isinstance(currentRoute, str):
            return "Creating route is not possible"

        #dotList = []
        sumCost = 0
        for idx, (x,y) in enumerate(currentRoute):

            #dont check next point if last point has been reached
            if idx == len(currentRoute)-1:
                break

            #set pointA and pointB, calculate difference
            pointA=currentRoute[idx]
            pointB=currentRoute[idx+1]
            differenceX = list(pointB)[0] - list(pointA)[0]
            differenceY = list(pointB)[1] - list(pointA)[1]

            cost, dots = determineCostAndDots(differenceX, differenceY)
            sumCost += cost
            #dotList.append(dots)
        heapq.heappush(routeList, (sumCost, currentRoute))
    bestRoute = heapq.heappop(routeList)[1]
    print("hello")
    return bestRoute





def astar(array, start, goal, shiftpos, startAxis, goalAxis, testingPath,testedPath, heuristicType, weight):
    if array[start] == 1:
        return "Start point is blocked and therefore goal cant be reached"
    elif array[goal] == 1:
        return "Goal point is blocked and therefore cant be reached"


    close_set = set()

    came_from = {}

    gscore = {start: 0}

    fscore = {start: heuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
    count = 0
    neiCount = 0

    while oheap:
        current = heapq.heappop(oheap)[1]

        if testingPath == True:
            if count > 0:
                currentBox.obj.color = vector(0,0.5,0)
                currentBox.obj.opacity = 0.5
                #currentBox.obj.delete()
                currentBox=Objects.currentDebugBox((current[0]+1, current[1]+1))
            else:
                currentBox=Objects.currentDebugBox((current[0]+1, current[1]+1))
                count +=1


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

            data = []

            while current in came_from:
                data.append(current)
                current = came_from[current]


            return data

        close_set.add(current) #add the from oheap popped coordinate to the closed list

        nextNeighbors = determineNeighbors(current,start, goal, goalAxis, shiftpos)
        if testingPath == True:
            if neiCount > 0:
                neighBox.obj.visible = False
                neighBox.obj.delete()
                neiCount = 0

        for i, j in nextNeighbors:

            current_neighbor = i, j
            neighbor = current[0] + i, current[1] + j



            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            # if isRestricted(prevCurrdifference, current_neighbor):
            #     continue

            #if the neighbour is in the closed set and the G score is greater than the G score's for that position
            # then ignore and continue the loop
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            #if current neighbor is above shiftpos and current position is smaller than shiftpos, skip
            if dimensionShiftViolated(j, shiftpos, current):
                continue

            #if current has reached shiftpos, then allow going vertical again
            if directionalRulesApply(current, current_neighbor, shiftpos):
                if current != start:
                    #restrict the way pipes can be placed according to rules
                    came_fromDifference = [abs(current[0] - came_from[current][0]), abs(current[1] - came_from[current][1])]
                    if isDirectionRestricted(came_fromDifference, current_neighbor):
                        continue
                elif startRestricted(current_neighbor, startAxis):
                     continue



            if 0 <= neighbor[0] < array.shape[0]:

                if 0 <= neighbor[1] < array.shape[1]:
                    # check if the neighbor is occupied (0 means not occupied, 1 means is occupied)
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue

                    #if neighbor is not occupied, check if space before it is occupied
                    elif abs(current[0]- neighbor[0]) >= 1:
                        if previousXoccupied(neighbor, current, array):
                            continue

                    # if neighbor is not occupied, check if space before it is occupied
                    elif abs(current[1] - neighbor[1]) >=1:
                        if previousYoccupied(current, neighbor, array):
                            continue



                else:

                    # array bound y walls

                    continue

            else:

                # array bound x walls

                continue




            #If the G score for the neighbour is less than the other G score's for that position OR if this neighbour
            # is not in the open list (i.e. a new, untested position) then update our lists and add to the open list
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                if testingPath == True:
                    if neiCount > 0:
                        neighBox.obj.visible = False
                        neighBox.obj.delete()
                        neighBox = Objects.neighborDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                        time.sleep(0.1)
                    else:
                        neighBox = Objects.neighborDebugBox((neighbor[0] + 1, neighbor[1] + 1))
                        neiCount += 1
                        time.sleep(0.1)

                if testedPath == True:
                    testedBox = Objects.possiblePositionDebugBox((neighbor[0] + 1, neighbor[1] + 1))

                came_from[neighbor] = current

                gscore[neighbor] = tentative_g_score

                fscore[neighbor] = tentative_g_score + artificialHeuristic(current, neighbor, goal, current_neighbor, heuristicType,weight)

                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return "Creating route is not possible"


