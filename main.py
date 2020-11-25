#this needs to get transferred to a tkinter menu

import Objects
from vpython import *
from win32api import GetSystemMetrics
import LogicalVfunctions as lvf
import astar as agt
import tkinter as tk
from tkinter import ttk
import math
import random



class App:
    def __init__(self):
        root = tk.Tk()
        #setting variables
        CameraOption = tk.StringVar()
        displayWallOption = tk.IntVar()
        displayTopOption = tk.IntVar()
        displayObstacleOption = tk.IntVar()
        displayPipesOption = tk.IntVar()
        infoOption = tk.IntVar()
        topAndWallxSizeString = tk.StringVar()
        topHeightString=tk.StringVar()
        wallHeightString=tk.StringVar()
        defaultOption = tk.IntVar()
        backgroundComboBoxString = tk.StringVar()
        resWidth = tk.StringVar()
        resHeight = tk.StringVar()
        coordinateInfoOption = tk.IntVar()
        showTestingPathsOption = tk.IntVar()
        showTestedPathsOption = tk.IntVar()
        randomizeOption = tk.IntVar()

        #functions
        def sendParameters():
            #parameters
            wallThickness = 15  # fixed value
            topAndWallxSize = float(topAndWallxSizeString.get())
            wallHeight = float(wallHeightString.get())
            topHeight = float(topHeightString.get())
            dot_distance = 10.5
            dot_distFromwall_x = 2.25
            dot_distFromWallBottom_y = 37.5
            dot_distToTopTop_y = 10.1
            wallShape = vector(topAndWallxSize, wallHeight, wallThickness)
            topShape = vector(topAndWallxSize, topHeight, wallThickness)
            x_dots = int(math.ceil((topAndWallxSize-2*dot_distFromwall_x)/dot_distance))
            y_dots = int(math.ceil((wallHeight+topHeight-dot_distFromWallBottom_y-dot_distToTopTop_y-wallThickness)/dot_distance))
            #wallToTopShiftDistance = wallHeight +wallThickness
            wallToTopShiftDots = int(math.ceil((wallHeight - dot_distFromWallBottom_y)/dot_distance))
            wallcolor = vector(0.8, 0.8, 0.8)
            topcolor = vector(0.8, 0.8, 0.8)
            dotcolor = color.black
            lampvisible = False
            #Options
            camera = setCamera(CameraOption.get(), wallShape)
            coordinateInfoVisible = coordinateInfoOption.get()
            wallVisible = displayWallOption.get()
            topVisible = displayTopOption.get()
            obstacleVisible = displayObstacleOption.get()
            pipeVisible= displayPipesOption.get()
            backgroundColor = setBackgroundColor(backgroundCombobox.get())
            testingPath = showTestingPathsOption.get()
            testedPath = showTestedPathsOption.get()
            if randomizeOption.get() == 0:
                level = levelCombobox.get()
            else: level = obstacleProbabilityCombobox.get()
            xRes = resWidth.get()
            yRes = resHeight.get()
            createScene(wallShape, topShape, wallThickness, wallcolor, topcolor, dotcolor, lampvisible, wallVisible,
                        topVisible, obstacleVisible, pipeVisible,
                        coordinateInfoVisible, camera, backgroundColor, x_dots, y_dots, dot_distFromwall_x,
                        dot_distFromWallBottom_y, dot_distance, wallToTopShiftDots,testingPath,testedPath,level,xRes,yRes)







        def setDefaultObjectParameters():
            if defaultOption.get() == 1:
                topAndWallxSizeString.set("100.0")
                topHeightString.set("115.0")
                wallHeightString.set("200.0")
                topAndWallxSizeEntry.config(state="disabled")
                topHeightEntry.config(state="disabled")
                wallHeightEntry.config(state="disabled")
            else:
                topAndWallxSizeEntry.config(state="enabled")
                topHeightEntry.config(state="enabled")
                wallHeightEntry.config(state="enabled")

        def setRandomizeLevel():
            if randomizeOption.get() == 1:
                levelCombobox.config(state="disabled")
                obstacleProbabilityCombobox.config(state="enabled")
            else:
                levelCombobox.config(state="enabled")
                obstacleProbabilityCombobox.config(state="disabled")


        def setCamera(Option, wallShape):
            if Option == "2DFront":
                camera = 0.5 * wallShape
                return camera
            elif Option == "2DUP":
                tk.Message.showinfo("Info", "Feature not implemented yet")
                return
            elif Option == "3D":
                tk.Message.showinfo("Info", "Feature not implemented yet")
                return

        def setBackgroundColor(Option):
            if Option=="white":
                backgroundColor = color.white
            else:
                backgroundColor = color.black
            return backgroundColor

        def calcDotCall():
            calcCurrentDots()
            root.after(100, calcDotCall)

        def calcCurrentDots():
            wallThickness = 15  # fixed value
            topAndWallxSize = float(topAndWallxSizeString.get())
            wallHeight = float(wallHeightString.get())
            topHeight = float(topHeightString.get())
            dot_distance = 10.5
            dot_distFromwall_x = 2.25
            dot_distFromWallBottom_y = 37.5
            dot_distToTopTop_y = 10.1
            x_dots = int(math.ceil((topAndWallxSize-2*dot_distFromwall_x)/dot_distance))
            y_dots = int(math.ceil((wallHeight+topHeight-dot_distFromWallBottom_y-dot_distToTopTop_y-wallThickness)/dot_distance))
            wallToTopShiftDots = int(math.ceil((wallHeight - dot_distFromWallBottom_y)/dot_distance))
            xDotShowLabel.configure(text="xDots: " + str(x_dots))
            yDotShowLabel.configure(text="yDots: " + str(y_dots))
            shiftAtShowLabel.configure(text= "shift at: " + str(wallToTopShiftDots))
            root.update()




        #setting title
        root.title("Pipe Lab")
        #setting window size
        width=1000
        height=600
        screenwidth = root.winfo_screenwidth()
        screenheight = root.winfo_screenheight()
        alignstr = '%dx%d+%d+%d' % (width, height, (screenwidth - width) / 2, (screenheight - height) / 2)
        root.geometry(alignstr)
        root.resizable(width=False, height=False)

        style = ttk.Style()
        style.configure("TButton", font = ("Calibri", 10), justify = "center")
        style.configure("TRadiobutton", font = ("Calibri", 10),justify="left", anchor = "w", width = 20)
        style.configure("TCheckbutton", font = ("Calibri", 10), justify="left", anchor = "w", width = 20)
        style.configure("Nowidth.TCheckbutton", font = ("Calibri", 10), justify="left", anchor = "w")
        style.configure("TLabel", font = ("Calibri", 12), justify="left")
        style.configure("Res.TLabel", font=("Calibri", 12), justify="left", width=10)
        style.configure("TEntry", font = ("Calibri", 12), justify="center")
        style.configure("Res.TEntry", font = ("Calibri", 12), justify="center", width =10)
        style.configure("TFrame" , font = ("Calibri", 12), justify="center")


        #create button
        CreateSceneButton=ttk.Button(root, text="Create Scene", command=sendParameters)
        CreateSceneButton.place(x=320,y=440,width=170,height=43)

        # create Object Parameters
        objectParameterLabel=ttk.Label(root, text = "Object Parameters:")
        objectParameterLabel.grid(row=0,column=0)

        displayWallCheckButton=ttk.Checkbutton(root, text= "Display Front Wall", variable = displayWallOption)
        displayWallCheckButton.grid(row=1,column=0)

        displayTopCheckButton=ttk.Checkbutton(root, text= "Display Top Wall", variable = displayTopOption)
        displayTopCheckButton.grid(row=2,column=0)

        displayObstacleCheckButton=ttk.Checkbutton(root, text= "Display Obstacles", variable = displayObstacleOption)
        displayObstacleCheckButton.grid(row=3,column=0)

        displayPipesCheckButton=ttk.Checkbutton(root, text= "Display Pipes", variable = displayPipesOption)
        displayPipesCheckButton.grid(row=4,column=0)

        topAndWallxSizeLabel=ttk.Label(root, text = "Width:")
        topAndWallxSizeLabel.grid(row=5,column=0)

        topAndWallxSizeEntry = ttk.Entry(root, textvariable = topAndWallxSizeString)
        topAndWallxSizeEntry.grid(row =6, column = 0)

        topHeightLabel=ttk.Label(root, text = "TopHeight:")
        topHeightLabel.grid(row=7,column=0)

        topHeightEntry = ttk.Entry(root, textvariable = topHeightString)
        topHeightEntry.grid(row=8, column = 0)

        wallHeightLabel=ttk.Label(root, text = "wallHeight:")
        wallHeightLabel.grid(row=9,column=0)

        wallHeightEntry = ttk.Entry(root, textvariable = wallHeightString)
        wallHeightEntry.grid(row=10, column = 0)

        defaultParametersButton = ttk.Checkbutton(root, text="Set default Values", command=setDefaultObjectParameters, variable = defaultOption)
        defaultParametersButton.grid(row=11, column=0)


        #camera field
        CameraLabel=ttk.Label(root, text = "Camera Parameters:")
        CameraLabel.grid(row=0,column=1)

        TwoDUpViewOption=ttk.Radiobutton(root, text ="2D Up View", variable = CameraOption, value= "2DUp")
        TwoDUpViewOption.grid(row=2,column=1)

        TwoDFrontCamOption=ttk.Radiobutton(root, text ="2D Front View", variable = CameraOption, value= "2DFront")
        TwoDFrontCamOption.grid(row=1,column=1)

        ThreeDViewOption=ttk.Radiobutton(root, text ="3D View", variable = CameraOption, value= "3D")
        ThreeDViewOption.grid(row=3,column=1)

        #scene field
        sceneLabel= ttk.Label(root, text = "Scene Parameters:")
        sceneLabel.grid(row=0,column=2)

        backgroundCombobox = ttk.Combobox(root, values=["black", "white"])
        backgroundCombobox.grid(row=1, column=2)

        resolutionLabelW= ttk.Label(root, text = "Width:")
        resolutionLabelW.grid(row=2, column=2)

        resolutionEntryW = ttk.Entry(root, textvariable = resWidth)
        resolutionEntryW.grid(row=3, column=2)

        resolutionLabelH = ttk.Label(root, text = "Height:")
        resolutionLabelH.grid(row=4,column=2)

        resolutionEntryH = ttk.Entry(root, textvariable = resHeight)
        resolutionEntryH.grid(row=5, column=2)

        #info field

        infoLabel= ttk.Label(root, text = "Info Parameters:")
        infoLabel.grid(row=0,column=3)

        CoordinateInfoCheckButton=ttk.Checkbutton(root, text= "Show coordinate info", variable = coordinateInfoOption)
        CoordinateInfoCheckButton.grid(row=1,column=3)

        #goal parameter field
        #todo: set start and goal parameters based on dots

        #parameterOutputField
        parameterOutputFrame = ttk.Frame(root)
        parameterOutputFrame.place(x=350,y=340)

        parameterOutputLabel = ttk.Label(parameterOutputFrame, text="Parameter Output:")
        parameterOutputLabel.grid(row=0,column=0)

        xDotShowLabel = ttk.Label(parameterOutputFrame,text="xDots: ")
        xDotShowLabel.grid(row=1, column=0)

        yDotShowLabel = ttk.Label(parameterOutputFrame,text="yDots: ")
        yDotShowLabel.grid(row=2, column=0)

        shiftAtShowLabel = ttk.Label(parameterOutputFrame,text="shift at: ")
        shiftAtShowLabel.grid(row=3, column=0)

        #fixme: round by multiple of 10.5

        #LevelSelect

        levelSelectLabel= ttk.Label(root, text = "Level Select:")
        levelSelectLabel.grid(row=0,column=4)

        levelCombobox = ttk.Combobox(root, values=["Level 1 (Easy)", "Level 2 (Medium)", "Level 3 (Hard)", "Random (Very hard)", "Debug Long"])
        levelCombobox.grid(row=1, column=4)

        randomizeCheckButton=ttk.Checkbutton(root, text= "Create random Level", variable =randomizeOption, command = setRandomizeLevel, style = "Nowidth.TCheckbutton")
        randomizeCheckButton.grid(row=2,column=4)

        obstacleProbabilityLabel= ttk.Label(root, text = "Obstacle frequency: ")
        obstacleProbabilityLabel.grid(row=3,column=4)

        obstacleProbabilityCombobox = ttk.Combobox(root, values=["Low", "Medium", "High"])
        obstacleProbabilityCombobox.grid(row=4, column=4)

        # debugField
        debugOptionsLabel= ttk.Label(root, text = "Debug options:")
        debugOptionsLabel.grid(row=0,column=5)

        showTestingPathsCheckButton=ttk.Checkbutton(root, text= "Show path finding", variable = showTestingPathsOption, style = "Nowidth.TCheckbutton")
        showTestingPathsCheckButton.grid(row=1,column=5)

        showTestedPathsCheckButton=ttk.Checkbutton(root, text= "Show tested positions", variable = showTestedPathsOption, style = "Nowidth.TCheckbutton")
        showTestedPathsCheckButton.grid(row=2,column=5)





        #insert default values
        topAndWallxSizeEntry.insert(0, "100.0")
        topHeightEntry.insert(0, "115.0")
        wallHeightEntry.insert(0, "200.0")
        backgroundCombobox.set("white")
        levelCombobox.set("Level 1 (Easy)")
        TwoDFrontCamOption.invoke()
        resolutionEntryW.insert(0, GetSystemMetrics(0))
        resolutionEntryH.insert(0, GetSystemMetrics(1))
        displayPipesCheckButton.invoke()
        displayTopCheckButton.invoke()
        displayWallCheckButton.invoke()
        displayObstacleCheckButton.invoke()
        obstacleProbabilityCombobox.config(state="disabled")

        #call functions that check after some time
        calcDotCall()


        root.mainloop()











    #objectcommands
    def CreateSceneButton_command(self):
        print("command")

    #functions

def create_Route(xDots, yDots, start, end, wallToTopShiftDots, startAxis, goalAxis,testingPath,testedPath):
    route = agt.displayPlot_Call(xDots, yDots, start, end, wallToTopShiftDots, startAxis, goalAxis,testingPath,testedPath)
    if isinstance(route, list):
        for idx,(x,y) in enumerate(route):
             route[idx] = (x+1,y+1)
        return route

def determineType(x,y):
    if x == 7 or y == 7:
        type="red+red"
    elif x == 6 or y == 6:
        type="red+yellow"
    elif x == 5 or y == 5:
        type="yellow+yellow"
    elif x == 3 or y == 3:
        type="blue"
    elif x == 2 or y == 2:
        type ="green"
    elif x == 1 or y == 1:
        type = "purple"
    else:
        type = "error"
        print("type doesnt exist")

    return type

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

def determineCorner(previousAxis, axis):
    if previousAxis == lvf.up and axis == lvf.up:
        cAxis = lvf.totop
    elif previousAxis == lvf.down and axis == lvf.down:
        cAxis = lvf.totop
    elif previousAxis == lvf.right and axis == lvf.up:
        cAxis = lvf.lefttoup
    elif previousAxis == lvf.left and axis == lvf.up:
        cAxis = lvf.righttoup
    elif previousAxis == lvf.right and axis == lvf.down:
        cAxis = lvf.downtoleft
    elif previousAxis == lvf.left and axis == lvf.down:
        cAxis = lvf.downtoright
    elif previousAxis == lvf.down and axis == lvf.right:
        cAxis = lvf.righttoup
    elif previousAxis == lvf.down and axis == lvf.left:
        cAxis = lvf.lefttoup
    elif previousAxis == lvf.up and axis == lvf.right:
        cAxis = lvf.downtoright
    elif previousAxis == lvf.up and axis == lvf.left:
        cAxis = lvf.downtoleft

    else:
        cAxis = lvf.up
        print("Error: CornerType cant be determined")
    return cAxis

def pipeBuilder(cRoute, pipeVisible, start, startAxis, goal, goalAxis, wallToTopShiftDots, wallVisible, topVisible):
    for idx, (x,y) in enumerate(cRoute):

        #dont check next point if last point has been reached
        if idx == len(cRoute)-1:
            break

        #set pointA and pointB, calculate difference
        pointA=cRoute[idx]
        pointB=cRoute[idx+1]
        differenceX = list(pointB)[0] - list(pointA)[0]
        differenceY = list(pointB)[1] - list(pointA)[1]

        if pointB[1] > wallToTopShiftDots and topVisible:
            pipeVisible = True
        elif pointB[1] <= wallToTopShiftDots and wallVisible:
            pipeVisible = True
        else:
            pipeVisible = False




        # if we process the first point, there cant be a previous one where an axis could be checked
        if idx >0:
            previousAxis = axis
            add, axis = determineAxis((differenceX, differenceY))
        else:
            add, axis = determineAxis((differenceX, differenceY))

        #determine type of pipe that is needed to close the distance between point a and b if horizontal
        if differenceX !=0 and pointA == start:
            type = determineType(abs(differenceX), abs(differenceY))
        elif differenceX !=0 and pointB == goal and axis == -goalAxis:
            type = determineType(abs(differenceX), abs(differenceY))
        elif differenceX !=0:
            type = determineType(abs(differenceX)-1, abs(differenceY))

        # determine type of pipe that is needed to close the distance between point a and b if vertical
        if differenceY !=0 and pointB == goal and axis == -goalAxis:
            type = determineType(abs(differenceX), abs(differenceY))
        elif differenceY !=0 and pointA == (x, wallToTopShiftDots) and \
                axis == lvf.up or differenceY !=0 and pointB == (x, wallToTopShiftDots) and axis == lvf.down:
            type = determineType(abs(differenceX), abs(differenceY) - 2)
        elif differenceY !=0 and pointA != start:
            type = determineType(abs(differenceX), abs(differenceY)- 1)
        elif differenceY !=0:
            type = determineType(abs(differenceX), abs(differenceY))

        if pointB == goal and pointA[0] == goal[0]:
            nearGoalHor = True
        elif pointB == goal and pointA[1] == goal[1]:
            nearGoalVert = True
        else:
            nearGoalHor = False
            nearGoalVert = False

        if pointA == start:
            Objects.pipe(type, (x,y), axis, pipeVisible)
            #fixme: add corner to wall instead of top
        elif pointA == (x, wallToTopShiftDots) and axis==lvf.up:
            Objects.pipe(type, (x + add[0], y + add[1]+1), axis, pipeVisible)
            cornerAxis= determineCorner(previousAxis, axis)
            corner = Objects.pipe("corner", (x,y), cornerAxis, pipeVisible)

            if cornerAxis == lvf.totop:
                corner.corner.rotate(angle=-0.5 * pi)  # rotate object
        # fixme: Doesnt work right
        elif nearGoalHor == True and goalAxis==lvf.right or nearGoalHor == True and goalAxis==lvf.left or \
                nearGoalVert == True and goalAxis == lvf.up or nearGoalVert == True and goalAxis == lvf.down:
            Objects.pipe(type, (x+add[0],y+add[1]), axis, pipeVisible)
            cornerAxis= determineCorner(previousAxis, axis)
            corner = Objects.pipe("corner", (x,y), cornerAxis, pipeVisible)
            cornerAxisEnd = determineCorner(axis, -goalAxis)
            cornerEnd = Objects.pipe("corner", (x+differenceX,y+differenceY), cornerAxisEnd, pipeVisible)
            if cornerAxis == lvf.totop:
                corner.corner.rotate(angle=-0.5 * pi)  # rotate object
        else:
            Objects.pipe(type, (x+add[0],y+add[1]), axis, pipeVisible)
            cornerAxis= determineCorner(previousAxis, axis)
            corner = Objects.pipe("corner", (x,y), cornerAxis, pipeVisible)
            if cornerAxis == lvf.totop:
                corner.corner.rotate(angle=-0.5 * pi)  # rotate object

def randomPrepInit(xDots,yDots, backgroundColor):
    #Initialise Start and Endpositions for random position
    possible_start_positions = [((1,1),lvf.up, lvf.upSG), ((6,1),lvf.up, lvf.upSG), ((xDots,1),lvf.up, lvf.upSG), ((1,3),lvf.right, lvf.rightSG), ((xDots,3),lvf.left, lvf.leftSG)]
    #possible_start_axis = {lvf.upTup, lvf.upTup, lvf.upTup, lvf.rightTup, lvf.leftTup}
    possible_goal_positions = [((1,yDots),lvf.down, lvf.downSG), ((6,yDots),lvf.down, lvf.downSG), ((10,yDots),lvf.down, lvf.downSG), ((1,yDots-3), lvf.right, lvf.rightSG), ((10,yDots-3),lvf.left, lvf.leftSG)]
    #possible_goal_axis = {lvf.downTup, lvf.downTup, lvf.downTup, lvf.rightTup, lvf.leftTup}

    randomSelectStart = random.randint(0, 4)
    randomSelectGoal = random.randint(0, 4)

    start = possible_start_positions[randomSelectStart][0]
    goal = possible_goal_positions[randomSelectGoal][0]
    startAxis = possible_start_positions[randomSelectStart][1]
    goalAxis = possible_goal_positions[randomSelectGoal][1]
    #axis + displacementVector
    startDirection = possible_start_positions[randomSelectStart][1] + possible_start_positions[randomSelectStart][2]
    goalDirection = possible_goal_positions[randomSelectGoal][1] + possible_goal_positions[randomSelectGoal][2]
    Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)
    PrepInitData = [start, goal, startAxis, goalAxis]
    return PrepInitData

def RandomLevelCreator(probability, wallToTopShiftDots, xDots, yDots, obsVisWall, obsVisTop):
    #create random Object on wall
    for i in range(3):
        #if random.random() <= probability:
        randPosX = random.randint(1, xDots-5)
        randPosY = random.randint(1, wallToTopShiftDots-5)
        randSizeX = random.randint(1, 5)
        randSizeY = random.randint(1, 5)
        Objects.obstacle((randSizeX, randSizeY), (randPosX, randPosY), obsVisWall)
        #else: continue

def createScene(wallShape, topShape, wallThickness, wallcolor, topcolor, dotcolor, lampvisible, wallVisible, topVisible,
            obstacleVisible, pipeVisible, coordinateInfoVisible, camera, backgroundColor, xDots, yDots, xGap, yGap,
            dotDist, wallToTopShiftDots,testingPath,testedPath,level,xRes, yRes):
    #create wall

    Objects.PipeLabInstance(wallShape, topShape, wallThickness, wallcolor, topcolor, lampvisible, wallVisible, topVisible,
            coordinateInfoVisible, camera, backgroundColor,xRes,yRes)
    #create logic matrix
    lvf.cdCm_Call(x_dots=xDots, y_dots=yDots, x_gap=xGap, y_gap=yGap, dot_dist=dotDist, wall_thickness=wallThickness,
                  dot_color=dotcolor, wall_to_top_shift_dots=wallToTopShiftDots, top_visible=topVisible, wall_visible=wallVisible)
    #"Level 1 (Easy)", "Level 2 (Medium)", "Level 3 (Hard)", "Random (Very hard)"
    #obstacle chronology go from bottom to top, left to right
    if wallVisible == True and obstacleVisible == True:
        obsVisWall = True
    else: obsVisWall = False

    if topVisible == True and obstacleVisible == True:
        obsVisTop = True
    else: obsVisTop = False

    random = False



    if level == "Level 1 (Easy)":
        # fixme: unfinished
        startAxis = lvf.up
        goalAxis = lvf.down
        startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
        goalDirection = goalAxis + vector(0, -4.5, 0)
        start = (6, 1)  # this will be a random vector along the wall or a manual input
        goal = (6, 25)  # this will be either a random vector along wall the top or a manual input
        Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)
        #wall
        obs1 = Objects.obstacle((4,9),(7,1), obsVisWall)
        obs2 = Objects.obstacle((5,3),(1,1), obsVisWall)
        obs3 = Objects.obstacle((3,3),(3,5), obsVisWall)
        obs3 = Objects.obstacle((4,4),(3,13), obsVisWall)
        #top
        obs4 = Objects.obstacle((7,3),(3,17), obsVisTop)
        obs5 = Objects.obstacle((2,2),(5,22), obsVisTop)
        random = False
    elif level == "Level 2 (Medium)":
        #fixme: unfinished
        startAxis = lvf.up
        goalAxis = lvf.right
        startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
        goalDirection = goalAxis + vector(6.5, 0, 0)
        start = (6, 1)  # this will be a random vector along the wall or a manual input
        goal = (1, 23)  # this will be either a random vector along wall the top or a manual input
        Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)
        obs1 = Objects.obstacle((1, 17), (1, 5), obsVisWall)
        obs1 = Objects.obstacle((3, 7), (1, 1), obsVisWall)
        #obs2 = Objects.obstacle(2, 5, (9, 1), obstacleVisible)
        #obs3 = Objects.obstacle(1, 13, (5, 5), obstacleVisible)

        obsx = Objects.obstacle((1, 2), (6, 7), obsVisWall)
        obsa = Objects.obstacle((1, 4), (8, 7), obsVisWall)
        obsb = Objects.obstacle((1, 2), (10, 7), obsVisWall)

        obs4 = Objects.obstacle((3, 2), (6, 11), obsVisWall)

        obsd = Objects.obstacle((2, 1), (9, 15), obsVisWall)

        obse = Objects.obstacle((3, 3), (2, 14), obsVisWall)

        # top:
        obs6 = Objects.obstacle((4, 1), (2, 17), obsVisTop)
        obs7 = Objects.obstacle((7, 1), (1, 20), obsVisTop)
        random = False
    elif level == "Level 3 (Hard)":
        startAxis = lvf.up
        goalAxis = lvf.right
        startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
        goalDirection = goalAxis + vector(6.5, 0, 0)
        start = (10, 1)  # this will be a random vector along the wall or a manual input
        goal = (1, 25)  # this will be either a random vector along wall the top or a manual input
        Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)

        #wall
        obs1 = Objects.obstacle((1, 2), (9, 1), obsVisWall)
        obs2 = Objects.obstacle((7, 1), (3, 4), obsVisWall)
        obs3 = Objects.obstacle((4, 3), (3, 6), obsVisWall)
        obs4 = Objects.obstacle((1, 7), (6, 10), obsVisWall)
        obs5 = Objects.obstacle((3, 7), (8, 10), obsVisWall)
        obs6 = Objects.obstacle((2, 4), (3, 13), obsVisWall)

        #top
        obs7 = Objects.obstacle((2, 4), (3, 17), obsVisTop)
        obs8 = Objects.obstacle((1, 3), (6, 17), obsVisTop)
        obs9 = Objects.obstacle((4, 1), (6, 20), obsVisTop)
        obs10 = Objects.obstacle((8, 4), (3, 22), obsVisTop)
        random = False
    elif level == "Debug Long":
        startAxis = lvf.up
        goalAxis = lvf.right
        startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
        goalDirection = goalAxis + vector(6.5, 0, 0)
        start = (1, 1)  # this will be a random vector along the wall or a manual input
        goal = (100, 25)  # this will be either a random vector along wall the top or a manual input
        Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)

        #wall
        obs1 = Objects.obstacle((1, 2), (9, 1), obsVisWall)
        obs2 = Objects.obstacle((7, 1), (3, 4), obsVisWall)
        obs3 = Objects.obstacle((4, 3), (3, 6), obsVisWall)
        obs4 = Objects.obstacle((1, 7), (6, 10), obsVisWall)
        obs5 = Objects.obstacle((3, 7), (8, 10), obsVisWall)
        obs6 = Objects.obstacle((2, 4), (3, 13), obsVisWall)

        #top
        obs7 = Objects.obstacle((2, 4), (3, 17), obsVisTop)
        obs8 = Objects.obstacle((1, 3), (6, 17), obsVisTop)
        obs9 = Objects.obstacle((4, 1), (6, 20), obsVisTop)
        obs10 = Objects.obstacle((8, 4), (3, 22), obsVisTop)
        random = False

    elif level == "Low":
        frequency = 3
        random = True
        PrepInitData = randomPrepInit(xDots,yDots, backgroundColor)





    # create start and end
    # startAxis = lvf.up
    # goalAxis = lvf.right
    # startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
    # goalDirection = goalAxis + vector(6.5, 0, 0)
    # start = (6, 1)  # this will be a random vector along the wall or a manual input
    # goal = (1, 24)  # this will be either a random vector along wall the top or a manual input
    # Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)
    #lvf.up: vector(0,4.5,0)
    #lvf.right: vector(6.5,0,0)
    # create objects
    # create obstacles

    #emptyLevel
    # startAxis = lvf.up
    # goalAxis = lvf.right
    # startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
    # goalDirection = goalAxis + vector(6.5, 0, 0)
    # start = (6, 1)  # this will be a random vector along the wall or a manual input
    # goal = (1, 24)  # this will be either a random vector along wall the top or a manual input
    # Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)


    #debug level1
    # startAxis = lvf.up
    # goalAxis = lvf.down
    # startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
    # goalDirection = goalAxis + vector(0, -4.5, 0)
    # start = (1, 1)  # this will be a random vector along the wall or a manual input
    # goal = (10, 25)  # this will be either a random vector along wall the top or a manual input
    # Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)
    # obs1 = Objects.obstacle(1, 16, (5, 1), obstacleVisible)
    # obs1 = Objects.obstacle(1, 7, (5, 17), obstacleVisible)

    #debug level2
    # startAxis = lvf.up
    # goalAxis = lvf.left
    # startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
    # goalDirection = goalAxis + vector(0, -4.5, 0)
    # start = (1, 1)  # this will be a random vector along the wall or a manual input
    # goal = (10, 1)  # this will be either a random vector along wall the top or a manual input
    # Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)
    # obs1 = Objects.obstacle(1, 17, (5, 1), obstacleVisible)

    #debug level 3
    # startAxis = lvf.up
    # goalAxis = lvf.left
    # startDirection = startAxis + vector(0, 4.5, 0)  # adding vector is a placeholder solution
    # goalDirection = goalAxis + vector(6.5, 0, 0)
    # start = (1, 1)  # this will be a random vector along the wall or a manual input
    # goal = (10, 25)  # this will be either a random vector along wall the top or a manual input
    # Objects.StartEndInt(start, goal, startDirection, goalDirection, backgroundColor)
    # obs1 = Objects.obstacle(9, 1, (1, 13), obstacleVisible)
    # obs2 = Objects.obstacle(8, 1, (2, 18), obstacleVisible)


    # Level 3 (doesnt look right... pls fix)
    # obs1 = Objects.obstacle(1,2,(9,1), obstacleVisible)
    # obs2 = Objects.obstacle(7,1,(3,3), obstacleVisible)
    # obs3 = Objects.obstacle(1,2,(9,4), obstacleVisible)
    # obs3 = Objects.obstacle(3,5,(3,6), obstacleVisible)
    # obs4 = Objects.obstacle(2,2,(3,13), obstacleVisible)
    # obs5 = Objects.obstacle(1,7,(3,9), obstacleVisible)
    # obs6 = Objects.obstacle(3,8,(7,9), obstacleVisible)
    # obs7 = Objects.obstacle(2,4,(3,17), obstacleVisible)
    # obs8 = Objects.obstacle(1,3,(6,17), obstacleVisible)
    # obs9 = Objects.obstacle(4,1,(6,20), obstacleVisible)

    # Level 4

    #
    # # 2.5
    # red1 = Objects.pipe("red", (6, 1), lvf.up, pipeVisible)
    # sock1 = Objects.pipe("socket", (6, 4), lvf.up, pipeVisible)
    # red2 = Objects.pipe("red", (6, 7), lvf.down, pipeVisible)
    # corn1 = Objects.pipe("corner", (6, 8), lvf.downtoleft, pipeVisible)
    # blue1 = Objects.pipe("blue", (5, 8), lvf.left, pipeVisible)
    # corn2 = Objects.pipe("corner", (2, 8), lvf.righttoup, pipeVisible)
    # red3 = Objects.pipe("red", (2, 9), lvf.up, pipeVisible)
    # sock2 = Objects.pipe("socket", (2, 12), lvf.up, pipeVisible)
    # red4 = Objects.pipe("red", (2, 15), lvf.down, pipeVisible)
    # corn4 = Objects.pipe("corner", (2, 17), lvf.fromwall, pipeVisible)
    # corn4.corner.rotate(angle=-0.5 * pi)  # rotate object
    # corn5 = Objects.pipe("corner", (2, 16), lvf.totop, pipeVisible)
    # corn5.corner.rotate(angle=-0.5 * pi)  # rotate object
    # yellow1 = Objects.pipe("yellow", (2, 18), lvf.up, pipeVisible)
    # sock3 = Objects.pipe("socket", (2, 20), lvf.up, pipeVisible)
    # red5 = Objects.pipe("red", (2, 23), lvf.down, pipeVisible)
    # corn6 = Objects.pipe("corner", (2, 24), lvf.downtoleft, pipeVisible)
    # purple1 = Objects.pipe("purple", (1, 24), lvf.right, pipeVisible)
    # # blue2 =Objects.pipe("blue", (3,24), lvf.left, pipeVisible)


    # calculate a* route
    cMatrix_route = ""
    if random == False:
        if pipeVisible == True:
            cMatrix_route = create_Route(xDots, yDots, start, goal, wallToTopShiftDots, startAxis, goalAxis,testingPath,testedPath)
            if isinstance(cMatrix_route, list):
                print(cMatrix_route)
                pipeBuilder(cMatrix_route, pipeVisible, start,startAxis, goal, goalAxis, wallToTopShiftDots, wallVisible, topVisible)
    elif random == True:
        start = PrepInitData[0]
        goal = PrepInitData[1]
        startAxis = PrepInitData[2]
        goalAxis = PrepInitData[3]

        while isinstance(cMatrix_route, list) == False:
            #fixme: remove old obstacles with every new instance
            lvf.cdCm_Call(x_dots=xDots, y_dots=yDots, x_gap=xGap, y_gap=yGap, dot_dist=dotDist,
                          wall_thickness=wallThickness,
                          dot_color=dotcolor, wall_to_top_shift_dots=wallToTopShiftDots, top_visible=topVisible,
                          wall_visible=wallVisible)
            RandomLevelCreator(frequency, wallToTopShiftDots, xDots, yDots, obsVisWall, obsVisTop)
            cMatrix_route = create_Route(xDots, yDots, start, goal, wallToTopShiftDots, startAxis, goalAxis,
                                         testingPath,
                                         testedPath)
        pipeBuilder(cMatrix_route, pipeVisible, start, startAxis, goal, goalAxis, wallToTopShiftDots, wallVisible,
                    topVisible)






# global dotCoordMatrix
# dotCoordMatrix = lvf.create_dotCoord_Matrix(x_dots=10, y_dots=25, z_dots=1, x_gap = 2.25, y_gap =37.5, dot_dist = 10.5, wall_thickness=wallThickness, dot_color=dotcolor)

#start the app
if __name__ == "__main__":
    scene = canvas()
    app = App()







# Objects.pipe("red", dotCoordMatrix(10, 4), lvf.left, dotCoordMatrix)
# #dotCoordMatrix = lvf.setOccupancy(redpipe, dotCoordMatrix)
# Objects.pipe("red", dotCoordMatrix(4, 4), lvf.right, dotCoordMatrix)
# Objects.pipe("blue", dotCoordMatrix(3,4), lvf.left, dotCoordMatrix)
# cursor = Objects.cursor(startDirection, start)
# cursor.Change(dotCoordMatrix(4,4), lvf.left)
# obstacle = Objects.obstacle(5,2,dotCoordMatrix(3,10), dotCoordMatrix)


#TODO: \
#      - create tkinter menu /'
#      - optional: make compound objects in top and flip top by 90°


