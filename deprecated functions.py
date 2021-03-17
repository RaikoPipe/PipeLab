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

# if type == "red+red":
#     self.pipe_length = 44
#     self.pipe_width = 5
#     self.overhang = 4.5
#     self.overhang2 = -4.5
#     self.dotlength= 7
#     self.cost = 5.87
#     self.realMeter = 0.718
#     self.secondDotDistance = 6
#     self.socketDotDistance = 3
#     self.pipe_pos2 = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, self.secondDotDistance)]
#     self.directionalOverhang1 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
#     self.directionalOverhang2 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang2,
#                                                                  self.pipe_width)
#     # if validPlacement(position, pipe_length) == True
#     redpipe1 = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang1, axis=pipe_axis,
#              size=vector(self.pipe_length, self.pipe_width, self.pipe_width),
#              color=color.red, visible=pipe_visible)
#     redpipe2= cylinder(pos=lvf.transformToVvector(self.pipe_pos2) + self.directionalOverhang2, axis=-pipe_axis,
#              size=vector(self.pipe_length, self.pipe_width, self.pipe_width),
#              color=color.red, visible=pipe_visible)
#     createSocket(pipe_coord, pipe_axis, self.socketDotDistance, pipe_visible)
#     lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
#     #print("red+red created at position: " + str(pipe_coord))
#     lvf.remember(redpipe1, pipeDict)
#     lvf.remember(redpipe2, pipeDict)
#     costDict.append(self.cost)
#     dotLengthDict.append(self.dotlength)
#     lengthDict.append(self.realMeter)
# elif type == "red+red":
#     self.pipe_length = 33.5
#     self.pipe_width = 5
#     self.overhang = 4.5
#     self.overhang2 = -4.5
#     self.dotlength= 7
#     self.cost = 5.87
#     self.realMeter = 0.718
#     self.secondDotDistance = 6
#     self.socketDotDistance = 3
#     self.pipe_pos2 = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, self.secondDotDistance)]
#     self.directionalOverhang1 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
#     self.directionalOverhang2 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang2,
#                                                                  self.pipe_width)
#     # if validPlacement(position, pipe_length) == True
#     redpipe1 = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang1, axis=pipe_axis,
#              size=vector(self.pipe_length, self.pipe_width, self.pipe_width),
#              color=color.red, visible=pipe_visible)
#     redpipe2= cylinder(pos=lvf.transformToVvector(self.pipe_pos2) + self.directionalOverhang2, axis=-pipe_axis,
#              size=vector(self.pipe_length, self.pipe_width, self.pipe_width),
#              color=color.red, visible=pipe_visible)
#     createSocket(pipe_coord, pipe_axis, self.socketDotDistance, pipe_visible)
#     lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
#     #print("red+red created at position: " + str(pipe_coord))
#     lvf.remember(redpipe1, pipeDict)
#     lvf.remember(redpipe2, pipeDict)
#     costDict.append(self.cost)
#     dotLengthDict.append(self.dotlength)
#     lengthDict.append(self.realMeter)
#
# elif type == "red+yellow":
#     self.pipe_length1 = 33.5
#     self.pipe_length2 = 23
#     self.pipe_width = 5
#     self.overhang = 4.5
#     self.overhang2= -4.5
#     self.dotlength= 6
#     self.cost= 5.65
#     self.realMeter = 0.613
#     self.secondDotDistance = 5
#     self.socketDotDistance = 3
#     self.pipe_pos2 = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, self.secondDotDistance)]
#     self.directionalOverhang1 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
#     self.directionalOverhang2 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang2,
#                                                                  self.pipe_width)
#     # if validPlacement(position, pipe_length) == True
#     redpipe1 = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang1, axis=pipe_axis,
#              size=vector(self.pipe_length1, self.pipe_width, self.pipe_width),
#              color=color.red, visible=pipe_visible)
#     yellowpipe2= cylinder(pos=lvf.transformToVvector(self.pipe_pos2) + self.directionalOverhang2, axis=-pipe_axis,
#              size=vector(self.pipe_length2, self.pipe_width, self.pipe_width),
#              color=color.yellow, visible=pipe_visible)
#     createSocket(pipe_coord, pipe_axis, self.socketDotDistance, pipe_visible)
#     lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
#     #print("red+yellow created at position: " + str(pipe_coord))
#     lvf.remember(redpipe1, pipeDict)
#     lvf.remember(yellowpipe2, pipeDict)
#     costDict.append(self.cost)
#     dotLengthDict.append(self.dotlength)
#     lengthDict.append(self.realMeter)
# elif type == "yellow+yellow":
#     self.pipe_length1 = 23
#     self.pipe_length2 = 23
#     self.pipe_width = 5
#     self.overhang = 4.5
#     self.overhang2 = -4.5
#     self.dotlength= 5
#     self.cost = 5.43
#     self.realMeter = 0.508
#     self.secondDotDistance = 4
#     self.socketDotDistance = 2
#     self.pipe_pos2 = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, self.secondDotDistance)]
#     self.directionalOverhang1 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
#     self.directionalOverhang2 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang2, self.pipe_width)
#
#     yellowpipe1 = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang1, axis=pipe_axis,
#              size=vector(self.pipe_length1, self.pipe_width, self.pipe_width),
#              color=color.yellow, visible=pipe_visible)
#     yellowpipe2= cylinder(pos=lvf.transformToVvector(self.pipe_pos2) + self.directionalOverhang2, axis=-pipe_axis,
#              size=vector(self.pipe_length2, self.pipe_width, self.pipe_width),
#              color=color.yellow, visible=pipe_visible)
#     createSocket(pipe_coord, pipe_axis, self.socketDotDistance, pipe_visible)
#     lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
#     #print("yellow+yellow created at position: " + str(pipe_coord))
#     lvf.remember(yellowpipe1, pipeDict)
#     lvf.remember(yellowpipe2, pipeDict)
#     costDict.append(self.cost)
#     dotLengthDict.append(self.dotlength)
#     lengthDict.append(self.realMeter)
# elif type == "red+pink":
#     self.pipe_length1 = 33.5
#     self.pipe_length2 = 12.5
#     self.pipe_width = 5
#     self.overhang = 4.5
#     self.overhang2 = -4.5
#     self.dotlength= 5
#     self.cost = 5.21
#     self.realMeter = 0.416
#     self.secondDotDistance = 4
#     self.socketDotDistance = 3
#     self.pipe_pos2 = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, self.secondDotDistance)]
#     self.directionalOverhang1 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
#     self.directionalOverhang2 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang2, self.pipe_width)
#
#     redpipe1 = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang1, axis=pipe_axis,
#              size=vector(self.pipe_length1, self.pipe_width, self.pipe_width),
#              color=color.red, visible=pipe_visible)
#     pinkpipe2= cylinder(pos=lvf.transformToVvector(self.pipe_pos2) + self.directionalOverhang2, axis=-pipe_axis,
#              size=vector(self.pipe_length2, self.pipe_width, self.pipe_width),
#              color=color.purple + vector(0.3,0.3,0.3), visible=pipe_visible)
#     createSocket(pipe_coord, pipe_axis, self.socketDotDistance, pipe_visible)
#     lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
#     #print("yellow+yellow created at position: " + str(pipe_coord))
#     lvf.remember(redpipe1, pipeDict)
#     lvf.remember(pinkpipe2, pipeDict)
#     costDict.append(self.cost)
#     dotLengthDict.append(self.dotlength)
#     lengthDict.append(self.realMeter)
# elif type == "pink+pink":
#     self.pipe_length1 = 12.5
#     self.pipe_length2 = 12.5
#     self.pipe_width = 5
#     self.overhang = 4.5
#     self.overhang2 = -4.5
#     self.dotlength= 3
#     self.cost = 4.99
#     self.realMeter = 0.311
#     self.secondDotDistance = 2
#     self.socketDotDistance = 1
#     self.pipe_pos2 = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, self.secondDotDistance)]
#     self.directionalOverhang1 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
#     self.directionalOverhang2 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang2, self.pipe_width)
#
#     pinkpipe1 = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang1, axis=pipe_axis,
#              size=vector(self.pipe_length1, self.pipe_width, self.pipe_width),
#              color=color.purple + vector(0.3,0.3,0.3), visible=pipe_visible)
#     pinkpipe2= cylinder(pos=lvf.transformToVvector(self.pipe_pos2) + self.directionalOverhang2, axis=-pipe_axis,
#              size=vector(self.pipe_length2, self.pipe_width, self.pipe_width),
#              color=color.purple + vector(0.3,0.3,0.3), visible=pipe_visible)
#     createSocket(pipe_coord, pipe_axis, self.socketDotDistance, pipe_visible)
#     lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
#     #print("yellow+yellow created at position: " + str(pipe_coord))
#     lvf.remember(pinkpipe1, pipeDict)
#     lvf.remember(pinkpipe2, pipeDict)
#     costDict.append(self.cost)
#     dotLengthDict.append(self.dotlength)
#     lengthDict.append(self.realMeter)