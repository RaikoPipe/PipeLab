import astar
from vpython import *
import LogicalVfunctions as lvf

def RandomLevelCreator(frequency, wallToTopShiftDots, xDots, yDots, obsVisWall, obsVisTop):
    #create random Object on wall
    for i in range(frequency):
        #if random.random() <= probability:
        randPosX = random.randint(1, xDots-1)
        randPosY = random.randint(1, wallToTopShiftDots-1)
        randSizeX = random.randint(1, 2)
        randSizeY = random.randint(1, 2)
        lvf.setOccO_Call(randSizeX,randSizeY, (randPosX,randPosY))
        #else: continue
    #create random Objects on Top
    for i in range(frequency -2):
        randPosX = random.randint(1, xDots - 1)
        randPosY = random.randint(wallToTopShiftDots+2, yDots-1)
        randSizeX = random.randint(1, 2)
        randSizeY = random.randint(1, 2)
        lvf.setOccO_Call(randSizeX,randSizeY, (randPosX,randPosY))

def randomPrepInit(xDots,yDots, backgroundColor, wallVisible, topVisible):
    #Initialise Start and Endpositions for random position
    possible_start_positions = [((2,1),lvf.up, lvf.upSG), ((6,1),lvf.up, lvf.upSG), ((xDots,1),lvf.up, lvf.upSG), ((1,2),lvf.right, lvf.rightSG), ((xDots,2),lvf.left, lvf.leftSG)]
    #possible_start_axis = {lvf.upTup, lvf.upTup, lvf.upTup, lvf.rightTup, lvf.leftTup}
    possible_goal_positions = [((1,yDots),lvf.down, lvf.downSG), ((6,yDots),lvf.down, lvf.downSG), ((10,yDots),lvf.down, lvf.downSG), ((1,yDots), lvf.right, lvf.rightSG), ((10,yDots),lvf.left, lvf.leftSG)]
    #possible_goal_axis = {lvf.downTup, lvf.downTup, lvf.downTup, lvf.rightTup, lvf.leftTup}

    randomSelectStart = random.randint(0, 4)
    randomSelectGoal = random.randint(0, 4)

    start = possible_start_positions[randomSelectStart][0]
    goal = possible_goal_positions[randomSelectGoal][0]
    #print("start set at:", start)
    #print("goal set at:", goal)
    startAxis = possible_start_positions[randomSelectStart][1]
    goalAxis = possible_goal_positions[randomSelectGoal][1]
    #axis + displacementVector
    startDirection = startAxis + possible_start_positions[randomSelectStart][2]
    goalDirection = goalAxis + possible_goal_positions[randomSelectGoal][2]
    PrepInitData = [start, goal, startAxis, goalAxis]
    return PrepInitData


frequency = 5
PrepInitData = randomPrepInit(10,25, color.white, True, True)
start = PrepInitData[0]
goal = PrepInitData[1]
startAxis = PrepInitData[2]
goalAxis = PrepInitData[3]

pipeDict = {2:20,4:20}

cMatrix_route, parts = astar.astar(array, start, goal, 16, startAxis, goalAxis, False, False, "normal", 0,0,25, pipeDict, False)
print(cMatrix_route)