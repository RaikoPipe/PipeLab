import LogicalVfunctions as lvf
from copy import deepcopy
import numpy

def pipeTypeDictEmpty(dict):
    availableParts = []
    for index, (key, count) in enumerate(dict.items()):
        if count > 0:
            if key in availableParts:
                continue
            else:
                availableParts.append(key)
    return availableParts

def determineAxis(direction):
    if direction[0] > 0:
        angle =  lvf.right
        add = (1,0)
    elif direction[0] < 0:
        angle = lvf.left
        add = (-1, 0)
    elif direction[1] > 0:
        angle = lvf.up
        add = (0, 1)
    elif direction[1] <0:
        angle = lvf.down
        add = (0, -1)
    else:
        print("error in determining axis of a pipe")
        angle = lvf.right
    return add, angle

def pipe_stock_check(route, pipeTypeDict, part_dict):
    availableParts = []
    pipeDict = deepcopy(pipeTypeDict)
    if not part_dict:
        availableParts = pipeTypeDictEmpty(pipeDict)
        return availableParts


    for idx, (x,y) in enumerate(route):

        #dont check next point if last point has been reached
        if idx == len(route)-1:
            break

        #set pointA and pointB, calculate difference
        pointB = route[idx+1]
        og_part = part_dict.get(pointB)

        pipeDict[og_part] = pipeDict[og_part] - 1
        availableParts = pipeTypeDictEmpty(pipeDict)

    return availableParts


def change_standard_neighbors(standardNeighbors, emptyList):
    new_standard_neighbors = standardNeighbors
    removetuples = []
    for index, value in enumerate(emptyList):
        removetuples.append((value+1, 0 ))
        removetuples.append((-value-1, 0))
        removetuples.append((0, value+1))
        removetuples.append((0, -value-1))


    for index, tuple in enumerate(removetuples):
        if tuple in new_standard_neighbors:
            new_standard_neighbors.remove(tuple)

    return new_standard_neighbors

def set_standard_neighbors(list):
    dict = {}
    for value in list:
        dict[(value+1,0)] = value
        dict[(-value-1, 0)] = value
        dict[(0,value+1)] = value
        dict[(0,-value-1)] = value
    return dict

def pathCheck(route, matrix):
    newMatrix = deepcopy(matrix)
    for index, v in enumerate(route):
        if index == len(route) - 1:
            break
        a = route[index]
        b = route[index+1]
        diffX = b[0] - a[0]
        diffY = b[1] - a[1]
        n = (diffX, diffY)
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
        for i in range(1, diff + 1):
            pos = (a[0] + addX * i, a[1] + addY * i)
            if newMatrix[pos] == 1:
                return True
            else: newMatrix[pos] = 1
    return False

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
    return x,y

def getAlteredMatrix(path, Matrix):
    newMatrix = deepcopy(Matrix)
    for index, v in enumerate(path):
        if index == len(path) - 1:
            break
        a = path[index]
        b = path[index + 1]
        n = (b[0] - a[0],b[1] - a[1])
        axis = getAxis(n)
        nLength = abs(n[0] - n[1])
        for i in range(1, nLength + 1):
            pos = (a[0] + axis[0] * i, a[1] + axis[1] * i)
            newMatrix[pos] = 2
    return newMatrix


