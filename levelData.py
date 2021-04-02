import object_classes
from vpython import vector
import LogicalVfunctions as lvf
import random

#select level based on user input
def selectLevel(level, backgroundColor, wallVisible, topVisible, obsVisWall, obsVisTop):

    if level == "Level 1 (Easy)":
        startAxis = lvf.up
        goalAxis = lvf.down
        startDirection = startAxis + vector(0,4.5 , 0)  # adding vector is a placeholder solution
        goalDirection = goalAxis + vector(0, -4.5, 0)
        start = (2, 1)  # this will be a random vector along the wall or a manual input
        goal = (1, 25)  # this will be either a random vector along wall the top or a manual input
        object_classes.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor, wallVisible, topVisible)
        #wall
        object_classes.obstacle((1, 1), (6, 3), True)
        object_classes.obstacle((1, 1), (8, 11), True)
        object_classes.obstacle((1, 1), (10, 7), True)
        object_classes.obstacle((1, 1), (9, 6), True)
        object_classes.obstacle((1, 1), (10, 12), True)
        object_classes.obstacle((1, 1), (5, 14), True)
        object_classes.obstacle((1, 1), (9, 14), True)
        object_classes.obstacle((1, 1), (5, 10), True)
        object_classes.obstacle((1, 1), (1, 15), True)
        object_classes.obstacle((1, 1), (2, 9), True)
        object_classes.obstacle((1, 1), (3, 20), True)
        object_classes.obstacle((1, 1), (3, 22), True)
        object_classes.obstacle((1, 1), (9, 19), True)
        object_classes.obstacle((1, 1), (4, 25), True)
        object_classes.obstacle((1, 1), (6, 24), True)
        object_classes.obstacle((1, 1), (6, 25), True)


        random = False
    elif level == "Level 2 (Medium)":
        #fixme: unfinished
        startAxis = lvf.up
        goalAxis = lvf.right
        startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
        goalDirection = goalAxis + lvf.rightSG
        start = (10, 1)  # this will be a random vector along the wall or a manual input
        goal = (1, 23)  # this will be either a random vector along wall the top or a manual input
        obs1 = object_classes.obstacle((1, 17), (1, 5), obsVisWall)
        obs1 = object_classes.obstacle((3, 7), (1, 1), obsVisWall)
        #obs2 = Objects.obstacle(2, 5, (9, 1), obstacleVisible)
        #obs3 = Objects.obstacle(1, 13, (5, 5), obstacleVisible)

        obsx = object_classes.obstacle((1, 2), (6, 7), obsVisWall)
        obsa = object_classes.obstacle((1, 4), (8, 7), obsVisWall)
        obsb = object_classes.obstacle((1, 2), (10, 7), obsVisWall)

        #obs4 = object_classes.obstacle((3, 2), (6, 11), obsVisWall)

        obsd = object_classes.obstacle((2, 1), (9, 15), obsVisWall)

        obse = object_classes.obstacle((3, 3), (2, 14), obsVisWall)

        # top:
        obs6 = object_classes.obstacle((4, 1), (2, 17), obsVisTop)
        obs7 = object_classes.obstacle((7, 1), (1, 20), obsVisTop)
        random = False
    elif level == "Level 3 (Hard)":
        startAxis = lvf.right
        goalAxis = lvf.up
        startDirection = startAxis + lvf.rightSG  # adding vector is a placeholder solution
        goalDirection = goalAxis + lvf.upSG
        start = (1, 2)  # this will be a random vector along the wall or a manual input
        goal = (6, 25)  # this will be either a random vector along wall the top or a manual input

        object_classes.obstacle((1, 1), (3, 11), True)
        object_classes.obstacle((1, 1), (5, 15), True)
        object_classes.obstacle((1, 1), (1, 9), True)
        object_classes.obstacle((1, 1), (7, 10), True)
        object_classes.obstacle((1, 1), (6, 7), True)
        object_classes.obstacle((1, 1), (9, 16), True)
        object_classes.obstacle((1, 1), (10, 14), True)
        object_classes.obstacle((1, 1), (10, 15), True)
        object_classes.obstacle((1, 1), (5, 10), True)
        object_classes.obstacle((1, 1), (3, 10), True)
        object_classes.obstacle((1, 1), (7, 20), True)
        object_classes.obstacle((1, 1), (9, 22), True)
        object_classes.obstacle((1, 1), (5, 21), True)
        object_classes.obstacle((1, 1), (9, 20), True)
        object_classes.obstacle((1, 1), (8, 19), True)
        object_classes.obstacle((1, 1), (3, 20), True)


    # elif level == "High Complexity":
    #     startAxis = lvf.up
    #     goalAxis = lvf.down
    #     startDirection = startAxis + lvf.upSG  # adding vector is a placeholder solution
    #     goalDirection = goalAxis + lvf.downSG
    #     start = (10, 1)  # this will be a random vector along the wall or a manual input
    #     goal = (1, 25)  # this will be either a random vector along wall the top or a manual input
    #     Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor, wallVisible, topVisible)
    #
    #     #wall
    #     obs1 = Objects.obstacle((1, 7), (5, 4), obsVisWall)
    #     obs2 = Objects.obstacle((3, 1), (8, 5), obsVisWall)
    #     obs3 = Objects.obstacle((3, 1), (2, 7), obsVisWall)
    #     obs4 = Objects.obstacle((1, 3), (2, 12), obsVisWall)
    #     obs5 = Objects.obstacle((1, 5), (6, 12), obsVisWall)
    #     obs6 = Objects.obstacle((3, 2), (8, 15), obsVisWall)
    #     obs7 = Objects.obstacle((4, 1), (2, 16), obsVisWall)
    #
    #     #top
    #     obs7 = Objects.obstacle((2, 1), (6, 18), obsVisTop)
    #     obs8 = Objects.obstacle((1, 5), (8, 18), obsVisTop)
    #     obs9 = Objects.obstacle((3, 3), (2, 22), obsVisTop)
    #     obs10 = Objects.obstacle((3, 1), (6, 24), obsVisTop)
    #     random = False


    elif level == "Save 1":
        startAxis = lvf.right
        goalAxis = lvf.down
        startDirection = startAxis + lvf.rightSG
        goalDirection = goalAxis + lvf.downSG
        start = (1, 2)  # this will be a random vector along the wall or a manual input
        goal = (1, 25)  # this will be either a random vector along wall the top or a manual input
        object_classes.obstacle((1, 1), (5, 6), True)
        object_classes.obstacle((1, 1), (5, 1), True)
        object_classes.obstacle((1, 1), (1, 14), True)
        object_classes.obstacle((1, 1), (5, 14), True)
        object_classes.obstacle((1, 1), (4, 15), True)
        object_classes.obstacle((1, 1), (6, 8), True)
        object_classes.obstacle((1, 1), (5, 2), True)
        object_classes.obstacle((1, 1), (4, 13), True)
        object_classes.obstacle((1, 1), (7, 13), True)
        object_classes.obstacle((1, 1), (7, 1), True)
        object_classes.obstacle((1, 1), (3, 5), True)
        object_classes.obstacle((1, 1), (7, 2), True)
        object_classes.obstacle((1, 1), (7, 3), True)
        object_classes.obstacle((1, 1), (10, 11), True)
        object_classes.obstacle((1, 1), (10, 1), True)
        object_classes.obstacle((1, 1), (7, 9), True)
        object_classes.obstacle((1, 1), (2, 7), True)
        object_classes.obstacle((1, 1), (6, 16), True)
        object_classes.obstacle((1, 1), (8, 14), True)
        object_classes.obstacle((1, 1), (3, 2), True)
        object_classes.obstacle((1, 1), (3, 1), True)
        object_classes.obstacle((1, 1), (9, 10), True)
        object_classes.obstacle((1, 1), (4, 7), True)
        object_classes.obstacle((1, 1), (5, 13), True)
        object_classes.obstacle((1, 1), (8, 2), True)
        object_classes.obstacle((1, 1), (4, 12), True)
        object_classes.obstacle((1, 1), (10, 14), True)
        object_classes.obstacle((1, 1), (9, 5), True)
        object_classes.obstacle((1, 1), (4, 16), True)
        object_classes.obstacle((1, 1), (4, 8), True)
        object_classes.obstacle((1, 1), (8, 21), True)
        object_classes.obstacle((1, 1), (1, 20), True)
        object_classes.obstacle((1, 1), (4, 21), True)
        object_classes.obstacle((1, 1), (9, 23), True)
        object_classes.obstacle((1, 1), (7, 25), True)
        object_classes.obstacle((1, 1), (3, 20), True)
        object_classes.obstacle((1, 1), (6, 23), True)
        object_classes.obstacle((1, 1), (8, 25), True)
        object_classes.obstacle((1, 1), (3, 24), True)
        object_classes.obstacle((1, 1), (6, 25), True)
        object_classes.obstacle((1, 1), (5, 19), True)
        object_classes.obstacle((1, 1), (7, 23), True)
        object_classes.obstacle((1, 1), (5, 18), True)
        object_classes.obstacle((1, 1), (5, 24), True)
        object_classes.obstacle((1, 1), (7, 24), True)
        object_classes.obstacle((1, 1), (4, 19), True)
        object_classes.obstacle((1, 1), (9, 25), True)
        object_classes.obstacle((1, 1), (8, 18), True)


    elif level == "Save 2":
        startAxis = lvf.right
        goalAxis = lvf.right
        startDirection = startAxis + lvf.rightSG
        goalDirection = goalAxis + lvf.rightSG
        start = (1, 2)  # this will be a random vector along the wall or a manual input
        goal = (1, 25)  # this will be either a random vector along wall the top or a manual input
        object_classes.obstacle((1, 1), (6, 14), True)
        object_classes.obstacle((1, 1), (6, 15), True)
        object_classes.obstacle((1, 1), (6, 16), True)
        object_classes.obstacle((1, 1), (4, 13), True)
        object_classes.obstacle((1, 1), (5, 15), True)
        object_classes.obstacle((1, 1), (8, 2), True)
        object_classes.obstacle((1, 1), (5, 14), True)
        object_classes.obstacle((1, 1), (7, 2), True)
        object_classes.obstacle((1, 1), (1, 11), True)
        object_classes.obstacle((1, 1), (3, 8), True)
        object_classes.obstacle((1, 1), (5, 8), True)
        object_classes.obstacle((1, 1), (9, 2), True)
        object_classes.obstacle((1, 1), (10, 3), True)
        object_classes.obstacle((1, 1), (7, 4), True)
        object_classes.obstacle((1, 1), (6, 1), True)
        object_classes.obstacle((1, 1), (8, 11), True)
        object_classes.obstacle((1, 1), (6, 11), True)
        object_classes.obstacle((1, 1), (10, 6), True)
        object_classes.obstacle((1, 1), (5, 13), True)
        object_classes.obstacle((1, 1), (4, 3), True)
        object_classes.obstacle((1, 1), (8, 10), True)
        object_classes.obstacle((1, 1), (3, 6), True)
        object_classes.obstacle((1, 1), (3, 16), True)
        object_classes.obstacle((1, 1), (10, 1), True)
        object_classes.obstacle((1, 1), (6, 10), True)
        object_classes.obstacle((1, 1), (10, 11), True)
        object_classes.obstacle((1, 1), (9, 9), True)
        object_classes.obstacle((1, 1), (5, 3), True)
        object_classes.obstacle((1, 1), (1, 6), True)
        object_classes.obstacle((1, 1), (9, 8), True)
        object_classes.obstacle((1, 1), (10, 24), True)
        object_classes.obstacle((1, 1), (6, 25), True)
        object_classes.obstacle((1, 1), (3, 20), True)
        object_classes.obstacle((1, 1), (4, 19), True)
        object_classes.obstacle((1, 1), (3, 25), True)
        object_classes.obstacle((1, 1), (9, 21), True)
        object_classes.obstacle((1, 1), (1, 20), True)
        object_classes.obstacle((1, 1), (7, 25), True)
        object_classes.obstacle((1, 1), (6, 18), True)
        object_classes.obstacle((1, 1), (4, 20), True)
        object_classes.obstacle((1, 1), (8, 25), True)
        object_classes.obstacle((1, 1), (6, 20), True)
        object_classes.obstacle((1, 1), (2, 24), True)
        object_classes.obstacle((1, 1), (4, 22), True)
        object_classes.obstacle((1, 1), (10, 18), True)
        object_classes.obstacle((1, 1), (8, 20), True)
        object_classes.obstacle((1, 1), (8, 23), True)
        object_classes.obstacle((1, 1), (3, 19), True)

    elif level == "Save 3":
        startAxis = lvf.up
        goalAxis = lvf.left
        startDirection = startAxis + lvf.upSG
        goalDirection = goalAxis + lvf.leftSG
        start = (6, 1)  # this will be a random vector along the wall or a manual input
        goal = (10, 25)  # this will be either a random vector along wall the top or a manual input
        object_classes.obstacle((2, 2), (2, 8), True)
        object_classes.obstacle((1, 2), (7, 2), True)
        object_classes.obstacle((2, 1), (9, 2), True)
        object_classes.obstacle((2, 1), (2, 13), True)
        object_classes.obstacle((2, 1), (6, 11), True)
        object_classes.obstacle((2, 2), (5, 19), True)
        object_classes.obstacle((1, 2), (8, 22), True)
        object_classes.obstacle((2, 1), (4, 18), True)

    elif level == "Save 4":
        startAxis = lvf.right
        goalAxis = lvf.left
        startDirection = startAxis + lvf.rightSG
        goalDirection = goalAxis + lvf.leftSG
        start = (1,2)
        goal = (10,25)

        object_classes.obstacle((1, 1), (1, 1), True)
        object_classes.obstacle((1, 1), (1, 8), True)
        object_classes.obstacle((1, 1), (6, 16), True)
        object_classes.obstacle((1, 1), (9, 3), True)
        object_classes.obstacle((1, 1), (6, 10), True)
        object_classes.obstacle((1, 1), (3, 10), True)
        object_classes.obstacle((1, 1), (2, 1), True)
        object_classes.obstacle((1, 1), (10, 12), True)
        object_classes.obstacle((1, 1), (5, 10), True)
        object_classes.obstacle((1, 1), (4, 11), True)
        object_classes.obstacle((1, 1), (3, 11), True)
        object_classes.obstacle((1, 1), (2, 8), True)
        object_classes.obstacle((1, 1), (1, 4), True)
        object_classes.obstacle((1, 1), (8, 3), True)
        object_classes.obstacle((1, 1), (6, 6), True)
        object_classes.obstacle((1, 1), (10, 3), True)
        object_classes.obstacle((1, 1), (7, 6), True)
        object_classes.obstacle((1, 1), (1, 3), True)
        object_classes.obstacle((1, 1), (3, 12), True)
        object_classes.obstacle((1, 1), (4, 6), True)
        object_classes.obstacle((1, 1), (7, 20), True)
        object_classes.obstacle((1, 1), (5, 25), True)
        object_classes.obstacle((1, 1), (8, 18), True)
        object_classes.obstacle((1, 1), (10, 23), True)
        object_classes.obstacle((1, 1), (5, 24), True)
        object_classes.obstacle((1, 1), (7, 18), True)
        object_classes.obstacle((1, 1), (9, 20), True)
        object_classes.obstacle((1, 1), (10, 24), True)
        object_classes.obstacle((1, 1), (3, 20), True)
        object_classes.obstacle((1, 1), (9, 24), True)
        object_classes.obstacle((1, 1), (8, 24), True)
        object_classes.obstacle((1, 1), (4, 23), True)
        #refresh = True
    else:
        start = False
        return start
    object_classes.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor, wallVisible, topVisible)
    return start, goal, startAxis, goalAxis


def selectRandomLevel(level):
    if level == "0: Empty":
        frequency = 0
    elif level == "1: Very Low":
        frequency = 10
    elif level == "2: Low":
        frequency = 20
    elif level == "3: Medium":
        frequency = 30
    elif level == "4: High":
        frequency = 40
    elif level == "5: Very High":
        frequency = 50
    elif level == "6: Extreme":
        frequency = 60
    elif level == "7: Very Extreme":
        frequency = 70
    elif level == "8: Ultimate":
        frequency = 80
    return frequency

def randomPrepInit(xDots,yDots, backgroundColor, wallVisible, topVisible):
    #Initialise Start and Endpositions for random position
    possible_start_positions = [((2,1),lvf.up, lvf.upSG), ((6,1),lvf.up, lvf.upSG), ((xDots,1),lvf.up, lvf.upSG), ((1,2),lvf.right, lvf.rightSG), ((xDots,2),lvf.left, lvf.leftSG)]
    possible_goal_positions = [((1,yDots),lvf.down, lvf.downSG), ((6,yDots),lvf.down, lvf.downSG), ((10,yDots),lvf.down, lvf.downSG), ((1,yDots), lvf.right, lvf.rightSG), ((10,yDots),lvf.left, lvf.leftSG)]

    randomSelectStart = random.randint(0, 4)
    randomSelectGoal = random.randint(0, 4)
    start = possible_start_positions[randomSelectStart][0]
    goal = possible_goal_positions[randomSelectGoal][0]
    startAxis = possible_start_positions[randomSelectStart][1]
    goalAxis = possible_goal_positions[randomSelectGoal][1]
    startDirection = startAxis + possible_start_positions[randomSelectStart][2]
    goalDirection = goalAxis + possible_goal_positions[randomSelectGoal][2]
    object_classes.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor, wallVisible, topVisible)
    PrepInitData = [start, goal, startAxis, goalAxis]
    object_classes.savedState = PrepInitData
    return PrepInitData