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

def bounds_violation(current, neighbor, array):
    diff = abs(abs(current[0] - neighbor[0]) - abs(current[1] - neighbor[1]))

    if 0 <= neighbor[0] < array.shape[0]:

        if 0 <= neighbor[1] < array.shape[1]:
            # check if the neighbor is occupied (0 means not occupied, 1 means is occupied)
            if array[neighbor[0]][neighbor[1]] == 1:
                return True

            # if neighbor is not occupied, check if space before it is occupied
            elif abs(current[0] - neighbor[0]) >= 1:
                if previousXoccupied(neighbor, current, array):
                    return True

            # if neighbor is not occupied, check if space before it is occupied
            elif abs(current[1] - neighbor[1]) >= 1:
                if previousYoccupied(current, neighbor, array):
                    return True
        else:
            return True # array bound y walls

    else:
        return True         # array bound x walls
    return False

def pipe_stock_check(route, start, goal, goalAxis, wallToTopShiftDots, pipeTypeDict):
    pipeDict = deepcopy(pipeTypeDict)
    emptyList = []
    start = (start[0]+1, start[1]+1)
    goal = (goal[0]+1, goal[1]+1)
    for idx, (x, y) in enumerate(route):
        route[idx] = (x + 1, y + 1)
    for idx, (x,y) in enumerate(route):

        #dont check next point if last point has been reached
        if idx == len(route)-1:
            break

        #set pointA and pointB, calculate difference
        pointA=route[idx]
        pointB=route[idx + 1]
        diffX=list(pointB)[0] - list(pointA)[0]
        diffY=list(pointB)[1] - list(pointA)[1]
        diff = abs(diffX-diffY)
        #og_part =

        if pointB == goal:
            print("point b is goal!")

        # if we process the first point, there cant be a previous one where an axis could be checked
        if idx >0:
            previousAxis = axis
            add, axis = determineAxis((diffX, diffY))
        else:
            add, axis = determineAxis((diffX, diffY))

        #determine type of pipe that is needed to close the distance between point a and b if horizontal

        # if differenceX !=0 and pointA == start:
        #     type, emptyList = determineType(abs(differenceX), abs(diff), pipeDict)
        # elif differenceX !=0 and pointB == goal and axis == -goalAxis:
        #     type, emptyList = determineType(abs(differenceX), abs(diff), pipeDict)
        # elif differenceX !=0:
        #     type, emptyList = determineType(abs(differenceX)-1, abs(diff), pipeDict)

        # determine type of pipe that is needed to close the distance between point a and b if vertical
        if diff !=0 and pointB == goal and axis == -goalAxis:
            emptyList = determineType(diff, pipeDict)
        elif diff !=0 and pointA == (x, wallToTopShiftDots) and \
                axis == lvf.up or diff !=0 and pointB == (x, wallToTopShiftDots) and axis == lvf.down:
            emptyList = determineType(diff - 2, pipeDict)
        elif diff !=0 and pointA != start:
            emptyList = determineType(diff-1, pipeDict)
        elif diff !=0:
            emptyList = determineType(diff, pipeDict)

    return emptyList

def determineType(diff, dict):

    if diff == 7:
        type="red+red"
        length = 7
    elif diff == 6:
         type="red+yellow"
         length= 6
    elif diff == 5:
         type="red+pink"
         length = 5
    elif diff == 4:
          type="long4"
          length = 4
    elif diff == 3:
         type="blue"
         length = 3
    elif diff == 2:
        type ="green"
        length = 2
    elif diff == 1:
         type = "purple"
         length = 1
    else:
        type = "error"
        print(diff,"type doesnt exist")

    dict[diff] = dict[diff] - 1
    emptyList = pipeTypeDictEmpty(dict)

    return emptyList