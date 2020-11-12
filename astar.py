import numpy as np
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import LogicalVfunctions as lvf


def displayPlot_Call(x,y, start, goal, shiftpos, startAxis, goalAxis):
    route_call = displayPlot(tuple(map(lambda c,k: c-k, start, (1,1))),tuple(map(lambda c,k: c-k, goal, (1,1))),x,y, shiftpos, startAxis, goalAxis)
    return route_call



def displayPlot(start_point,goal_point,x,y, shiftpos, startAxis, goalAxis):


    grid = lvf.glG_Call(x, y)  # 0s are positions we can travel on, 1s are walls(obstacles or already placed pipes)
    route = astar(grid, start_point, goal_point, shiftpos, startAxis, goalAxis)
    if isinstance(route, str):
        print(route)
    else:
        route = route + [start_point]
        route = route[::-1]
        print(route)

        # plot map and path
        x_coords = []
        y_coords = []
        for i in (range(0,len(route))):

            x = route[i][0]

            y = route[i][1]

            x_coords.append(x)
            y_coords.append(y)

        fig, ax = plt.subplots(figsize=(x, y))

        ax.imshow(grid, cmap=plt.cm.Dark2)

        ax.scatter(start_point[1], start_point[0], marker="*", color="green", s=200)

        ax.scatter(goal_point[1], goal_point[0], marker="*", color="orange", s=200)

        ax.plot(y_coords, x_coords, color="black")

        #plt.show()
        return route



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

#check if restriction that it needs to change direction by 90° after laying a pipe or pipe combo isn't violated.
def isRestricted(cameFromDifference, currentNeighbor):
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

# fixme: this function is a god damn clusterfuck. clean it up.
def determineNeighbors(current, goal, goalAxis, shiftpos):
    currToGoalDifference = (goal[0] - current[0], goal[1] - current[1])
    if current ==(5,15):
        print("here")

    if current == (current[0],shiftpos-1) or current == (current[0], shiftpos+1): # current is in shiftpos or shiftpos top
        if current[0] != goal[0] and current == (current[0],shiftpos-1): # we are not on same horizontal axis as goal
            neighbors = [(0, 9), (0, -8), (8, 0), (-8, 0),(0, 8), (0, -7), (7, 0), (-7, 0),(0, 7), (0, -6), (6, 0), (-6, 0),
                        (0, 5), (0, -4), (4, 0), (-4, 0),(0, 4), (0, -3), (3, 0), (-3, 0),(0, 3), (0, -2), (2, 0),
                        (-2, 0)]
            return neighbors
        #ixme: correct neighbors
        elif current[0] != goal[0] and current == (current[0],shiftpos+1): # we are not on same horizontal axis as goal
            neighbors = [(0, 8), (0, -9), (8, 0), (-8, 0),(0, 7), (0, -8), (7, 0), (-7, 0),(0, 6), (0, -7), (6, 0), (-6, 0),
                        (0, 4), (0, -5), (4, 0), (-4, 0),(0, 3), (0, -4), (3, 0), (-3, 0),(0, 2), (0, -3), (2, 0),
                        (-2, 0)]
        elif abs(currToGoalDifference[1]) <= 8 and current[0] == goal[0]: # goal is in reach and we are on the same axis
            #goal is on same horizontal axis
            if current[0] == goal[0] and currToGoalDifference[1] > 0 and currToGoalDifference[1] <= 8 and goalAxis == lvf.down:
                neighbors = [(0, 8), (0, 7), (0, 6), (0, 4), (0, 3), (0, 2)]
                return neighbors
            elif current[0] == goal[0] and currToGoalDifference[1] < 0 and currToGoalDifference[1] >= -8 and goalAxis == lvf.up:
                neighbors = [(0, -8), (0, -7), (0, -6), (0, -4), (0, -3), (0, -2)]
                return neighbors
        elif current[0] == goal [0] and abs(currToGoalDifference[1]) > 8: #we are on the same axis as goal, but its not reachable
            neighbors = [(8, 0), (-8, 0), (0, 8), (0, -8), (7, 0), (-7, 0), (0, 7), (0, -7), (6, 0),
                         (-6, 0),
                         (0, 5), (0, -5), (4, 0), (-4, 0), (0, 4), (0, -4), (3, 0), (-3, 0), (0, 3), (0, -3), (2, 0),
                         (-2, 0)]
            return neighbors





    #current is not in shiftpos but goal is in reach
    #goal is one same horizontal axis
    if current[0] == goal[0] and currToGoalDifference[1] > 0 and currToGoalDifference[1] <= 7 and goalAxis == lvf.down:
        neighbors = [(0, 7), (0, 6), (0, 5), (0, 3), (0, 2), (0, 1)]
    elif current[0] == goal[0] and currToGoalDifference[1] < 0 and currToGoalDifference[1] >= -7 and goalAxis == lvf.up:
        neighbors = [(0, -7), (0, -6), (0, -5), (0, -3), (0, -2), (0, -1)]
    #goal is on same vertical axis
    elif current[1] == goal[1] and currToGoalDifference[0] > 0 and currToGoalDifference[0] <= 7 and goalAxis == lvf.left:
        neighbors = [(7, 0), (6, 0), (5, 0), (3, 0), (2, 0), (1, 0)]
    elif current[1] == goal[1] and currToGoalDifference[0] < 0 and currToGoalDifference[0] >= 7 and goalAxis == lvf.right:
        neighbors = [(-7, 0), (-6, 0), (-5, 0), (-3, 0), (-2, 0), (-1, 0)]
    #goal is not in reach
    else: neighbors = [(0, 8), (0, -8), (8, 0), (-8, 0),(0, 7), (0, -7), (7, 0), (-7, 0),(0, 6), (0, -6), (6, 0), (-6, 0),
    (0, 4), (0, -4), (4, 0), (-4, 0),(0, 3), (0, -3), (3, 0), (-3, 0),(0, 2), (0, -2), (2, 0),
    (-2, 0)]

    return neighbors
    # currToGoalDifference = (goal[0]-current[0],goal[1]-current[1])
    # if current[1] == goal[1] and currToGoalDifference[0] > 0 and currToGoalDifference[0] <=7 and goalAxis == lvf.left:
    #     neighbors = [(7,0),(6,0),(5,0),(3,0),(2,0),(1,0)]
    # elif current[1] == goal[1] and currToGoalDifference[0] < 0 and currToGoalDifference[0] >=-7 and goalAxis == lvf.right:
    #     neighbors = [(-7,0),(-6,0),(-5,0),(-3,0),(-2,0),(-1,0)]

    # elif current[0] == goal[0] and currToGoalDifference[1] < 0 and currToGoalDifference[1] >=-7 and goalAxis == lvf.up:
    #     neighbors = [(-7,0),(-6,0),(-5,0),(-3,0),(-2,0),(-1,0)]
    # else: neighbors = [(0, 8), (0, -8), (8, 0), (-8, 0),(0, 7), (0, -7), (7, 0), (-7, 0),(0, 6), (0, -6), (6, 0), (-6, 0),
    # (0, 4), (0, -4), (4, 0), (-4, 0),(0, 3), (0, -3), (3, 0), (-3, 0),(0, 2), (0, -2), (2, 0),
    # (-2, 0)]


def heuristic(a,b): #fixme: how do i change it to base the score on cost effectiveness AND shortest path? skew the score a certain way?
    #manhattan distance
    distance = np.sqrt((b[0] - a[0])** 2  + (b[1] - a[1]) ** 2)
    return distance
    # manhattan distance: np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    # np.sqrt((b[0] - a[0])** 2  + (b[1] - a[1]) ** 2)
    #return distance

def goal_heuristic(a,b):
    #manhattan distance
    distance = np.sqrt((b[0] - a[0])** 2  + (b[1] - a[1]) ** 2)
    return distance
    # manhattan distance: np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    # np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    #return distance


#fixme: pipe placement needs to result in that place beeing occupied
# some error on start? places pipe that is 4 long
def astar(array, start, goal, shiftpos, startAxis, goalAxis):
    if array[start] == 1:
        return "Start point is blocked and therefore goal cant be reached"
    elif array[goal] == 1:
        return "Goal point is blocked and therefore cant be reached"

    #(that way, we can reach goal without having to place a corner)


    close_set = set()

    came_from = {}

    gscore = {start: 0}

    fscore = {start: goal_heuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
    count = 0

    while oheap:

        current = heapq.heappop(oheap)[1]



        if current == goal:

            data = []

            while current in came_from:
                data.append(current)
                current = came_from[current]

            return data

        close_set.add(current) #add the from oheap popped coordinate to the closed list

        nextNeighbors = determineNeighbors(current, goal, goalAxis, shiftpos)


        for i, j in nextNeighbors:

            current_neighbor = i,j
            neighbor = current[0] + i, current[1] + j

            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            # if isRestricted(prevCurrdifference, current_neighbor):
            #     continue

            #if current neighbor is above shiftpos and current position is smaller than shiftpos, skip
            if j > (shiftpos-1) - current[1] and current[1] < (shiftpos-1): #or j < current[1] - (shiftpos+1)  and current[1] > (shiftpos+1):
                continue

            #if current has reached shiftpos, then allow going vertical again
            if current != (current[0], shiftpos-1) or current == (current[0], shiftpos-1) and current_neighbor[0] != 0:
                if current != start:

                    #restrict the way pipes can be placed according to rules
                    came_fromDifference = [abs(current[0] - came_from[current][0]), abs(current[1] - came_from[current][1])]
                    if isRestricted(came_fromDifference, current_neighbor):
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

            #if the neighbour is in the closed set and the G score is greater than the G score's for that position
            # then ignore and continue the loop
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            #If the G score for the neighbour is less than the other G score's for that position OR if this neighbour
            # is not in the open list (i.e. a new, untested position) then update our lists and add to the open list
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:

                came_from[neighbor] = current

                gscore[neighbor] = tentative_g_score

                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return "Creating route is not possible"


#TODO: understand astar method
