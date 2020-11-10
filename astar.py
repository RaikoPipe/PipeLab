import numpy as np
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import LogicalVfunctions as lvf


def displayPlot_Call(x,y, start, goal, shiftpos):
    route_call = displayPlot(tuple(map(lambda c,k: c-k, start, (1,1))),tuple(map(lambda c,k: c-k, goal, (1,1))),x,y, shiftpos)
    return route_call



def displayPlot(start_point,goal_point,x,y, shiftpos):


    grid = lvf.glG_Call(x, y)  # 0s are positions we can travel on, 1s are walls(obstacles or already placed pipes)
    route = astar(grid, start_point, goal_point, shiftpos)
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

def heuristic(a,b): #fixme: how do i change it to base the score on cost effectiveness AND shortest path? skew the score a certain way?
    #manhattan distance
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    return distance
    # manhattan distance: np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    # np.sqrt((b[0] - a[0])** 2  + (b[1] - a[1]) ** 2)
    #return distance

def goal_heuristic(a,b): #fixme: how do i change it to base the score on cost effectiveness AND shortest path? skew the score a certain way?
    #manhattan distance
    distance = np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    return distance
    # manhattan distance: np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])
    # np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    #return distance

def astar(array, start, goal, shiftpos):
    if array[start] == 1:
        return "Start point is blocked and therefore goal cant be reached"
    elif array[goal] == 1:
        return "Goal point is blocked and therefore cant be reached"

    neighbors = [(0, 8), (0, -8), (8, 0), (-8, 0),(0, 7), (0, -7), (7, 0), (-7, 0),(0, 6), (0, -6), (6, 0), (-6, 0),
    (0, 4), (0, -4), (4, 0), (-4, 0),(0, 3), (0, -3), (3, 0), (-3, 0),(0, 2), (0, -2), (2, 0),
    (-2, 0)]# (purple:2,green:3,blue:4,yellow+yellow:6, red+yellow:7, red+red:8)

    
# fixme: option1:what happens if axis of current neighbor is the same as goal? Then goalneighbors must be used
# fixme: option2: after a pipe has been placed, corner must be placed in same axis, then place pipe etc...
    #(that way, we can reach goal without having to place a corner)


    close_set = set()

    came_from = {}

    gscore = {start: 0}

    fscore = {start: goal_heuristic(start, goal)}

    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
    count = 0

    while oheap:
        # if count > 0:
        #     previous = current
        #     current = heapq.heappop(oheap)[1]
        #     prevCurrdifference = [abs(current[0] - previous[0]), abs(current[1] - previous[1])]
        #
        #
        # else: #skip creating prevCurrDifference on first loop
        #     prevCurrdifference = [0,0]
        #     current = heapq.heappop(oheap)[1]

        current = heapq.heappop(oheap)[1]


        count +=1

        if current == goal:

            data = []

            while current in came_from:
                data.append(current)
                current = came_from[current]

            return data

        close_set.add(current) #add the from oheap popped coordinate to the closed list
        for i, j in neighbors:
            #todo: algorithm works fine, but it makes routes that violate rules
            #todo: option 1 :create heuristic that builds pipes and tries to stay as close to a* route as possible
            #todo: option 2 : after a* has explored all dots, create route and if rule is violated, try next best option (see at row 138) -> maybe restrict the way the route gets created rather than the neighbors?
            #todo: option 3 : maybe my currently commented code isnt wrong after all, but it doesnt know that it has to switch @y=16?
            #result: used came_from as fix




            current_neighbor = i,j
            neighbor = current[0] + i, current[1] + j

            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            # if isRestricted(prevCurrdifference, current_neighbor):
            #     continue

            #if current neighbor is above shiftpos and current position is smaller than shiftpos, skip
            if j > abs(current[1] - (shiftpos-1)) and current[1] < (shiftpos-1):
                continue


            #check if current has reached pos where dimension is changed

            if current != (current[0], shiftpos-1) or current == (current[0], shiftpos-1) and current_neighbor[0] != 0:
                #if current position is not start, check if neighbor would violate rules
                # if current_neighbor == (0,2):
                #     continue
                if current != start: #fixme: determine allowed neighbors with start axis(up,down,left,right...)

                    came_fromDifference = [abs(current[0] - came_from[current][0]), abs(current[1] - came_from[current][1])]
                    if isRestricted(came_fromDifference, current_neighbor):
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
#      allow only certain distances before astar must change by 90°
#     if current is near goal by 1 distance allow neighbor in form of (0, 1), (0, -1), (1, 0), (1, 0)
