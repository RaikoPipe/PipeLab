from copy import deepcopy

import object_classes
from vpython import *
from win32api import GetSystemMetrics
import LogicalVfunctions as lvf
import find_path as agt
import tkinter as tk
from tkinter import ttk
import math
import random
import matplotlib.pyplot as plt
from datetime import datetime
import levelData

# lengthText = Objects.displayText(vector(10,10,5))
# costText = Objects.displayText(vector(10,5,5))
savedState = []
costString = "P: "
lengthString = "C: "
partString = "Teile übrig: "
MinOString = "MinO: "
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
        displayWallDotsOption= tk.IntVar()
        displayTopDotsOption = tk.IntVar()
        gCoption = tk.StringVar()
        gPoption = tk.StringVar()
        gMinOoption = tk.StringVar()

        #partstrings
        pl1 = tk.StringVar()
        pc1 = tk.StringVar()
        pl2 = tk.StringVar()
        pc2 = tk.StringVar()
        pl3 = tk.StringVar()
        pc3 = tk.StringVar()
        pl4 = tk.StringVar()
        pc4 = tk.StringVar()
        pl5 = tk.StringVar()
        pc5 = tk.StringVar()





        def refreshPath():
            refresh = True
            sendParameters(refresh)

        def createNewScene():
            refresh = False
            sendParameters(refresh)

        def disableRefreshPathButton(event):
            refreshPathButton.config(state="disabled")

        def checkPipes(dict):

            if pl1.get() != "":
                dict[int(pl1.get())] = int(pc1.get())
            if pl2.get() != "":
                dict[int(pl2.get())] = int(pc2.get())
            if pl3.get() != "":
                dict[int(pl3.get())] = int(pc3.get())
            if pl4.get() != "":
                dict[int(pl4.get())] = int(pc4.get())
            if pl5.get() != "":
                dict[int(pl5.get())] = int(pc5.get())

            print(dict)

            return dict




        #functions
        def sendParameters(refresh):
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
            wallToTopShiftDots = int(math.ceil((wallHeight - dot_distFromWallBottom_y)/dot_distance))
            wallcolor = vector(0.8, 0.8, 0.8)
            topcolor = vector(0.8, 0.8, 0.8)
            dotcolor = color.black
            lampvisible = False
            #Options
            camera = setCamera(CameraOption.get(), wallShape, topShape)
            coordinateInfoVisible = coordinateInfoOption.get()
            wallVisible = displayWallOption.get()
            topVisible = displayTopOption.get()
            obstacleVisible = displayObstacleOption.get()
            pipeVisible= displayPipesOption.get()
            topDotVisible=displayTopDotsOption.get()
            wallDotVisible=displayWallDotsOption.get()
            backgroundColor = setBackgroundColor(backgroundCombobox.get())
            testingPath = showTestingPathsOption.get()
            testedPath = showTestedPathsOption.get()
            search_type = searchTypeCombobox.get()
            refreshObjectsButton.config(state="enabled")
            refreshObjectsButton.config(state="enabled")
            displayPipesCheckButton.config(state="enabled")
            displayTopCheckButton.config(state="enabled")
            displayWallCheckButton.config(state="enabled")
            displayObstacleCheckButton.config(state="enabled")
            displayWallDotsCheckButton.config(state="enabled")
            displayTopDotsCheckButton.config(state="enabled")
            refreshPathButton.config(state="enabled")
            pipeTypeDict = {}
            pipeTypeDict = checkPipes(pipeTypeDict)
            if randomizeOption.get() == 0:
                level = levelCombobox.get()
            else: level = obstacleProbabilityCombobox.get()
            xRes = resWidth.get()
            yRes = resHeight.get()

            gC = float(gCoption.get())
            gP = float(gPoption.get())
            gMinO = float(gMinOoption.get())
            refreshing = refresh
            if refresh == False:
                wallVisible = True
                topVisible = True
                obstacleVisible = True
                pipeVisible = True
                topDotVisible = True
                wallDotVisible = True

            # searchTypeList = ["astar", "best-first", "dijkstra"]
            # #write experiment data
            # successcounter = 0
            # for i in range(300):
            #     if successcounter >= 100:
            #         duration = 1000  # milliseconds
            #         freq = 440  # Hz
            #         winsound.Beep(freq, duration)
            #         break
            algList = []
            heuristicType = ""

            createScene(wallShape, topShape, wallThickness, wallcolor, topcolor, dotcolor, lampvisible, wallVisible,
                        topVisible, obstacleVisible, pipeVisible, topDotVisible, wallDotVisible,
                        coordinateInfoVisible, camera, backgroundColor, x_dots, y_dots, dot_distFromwall_x,
                        dot_distFromWallBottom_y, dot_distance, wallToTopShiftDots,testingPath,testedPath,level,xRes,yRes
                        , heuristicType, refreshing, pipeTypeDict, search_type,gC,gP,gMinO)
            global dotLengthText
            global costText
            global partText
            try:
                dotLengthText.visible = False
                dotLengthText.delete()
                costText.visible = False
                costText.delete()
                partText.visible = False
                partText.delete()
            except: Exception
            searchTypeText = label(text=search_type, pos=vector(110, 0, 5), align="left", color=color.white, linewidth=3,
                             background=color.black, height=15, opacity=1)
            dotLengthText = label(text=lengthString, pos = vector(120,0,5), align="left", color=color.white, linewidth=3, background=color.black, height = 15, opacity = 1)
            costText = label(text=costString, pos = vector(130,0,5), align="left", color=color.white, linewidth=3, background=color.black , height = 15, opacity = 1)
            MinOText = label(text=MinOString + str(MinOValue), pos = vector(140,0,5), align="left", color=color.white, linewidth=3, background=color.black , height = 15, opacity = 1)
            partText = label(text=partString, pos = vector(150,0,5), align="left", color=color.white, linewidth=3, background=color.black , height = 15, opacity = 1)

            MinOLabel.config(text="MinO: " + str(MinOValue))
            costCounter = 0
            dotCounter = 0
            for count, cost in enumerate(object_classes.costDict):
                costCounter += cost
            for count, dots in enumerate(object_classes.dotLengthDict):
                dotCounter += dots
            tempList = [search_type, dotCounter, costCounter]
            algList.append(tempList)
            searchNote = open("searchtype.txt", "a")
            lengthNote = open("length.txt", "a")
            costNote = open("cost.txt", "a")
            MinONote = open("MinONote.txt", "a")
            partNote = open("partNote.txt", "a")
            if refresh == False:
                searchNote.write("\n")
                lengthNote.write("\n")
                costNote.write("\n")
                MinONote.write("\n")
                partNote.write("\n")
            for y in algList:
                searchNote.write(y[0] + "\n")
                lengthNote.write(str(y[1]) + "\n")
                costNote.write(str(round(y[2],2)) + "\n")
                MinONote.write(str(MinOValue) + "\n")
                partNote.write(str(pipeCounter) + "\n")

        def refreshDisplayObjectsPrep():
            wallVisible = displayWallOption.get()
            topVisible = displayTopOption.get()
            obstacleVisible = displayObstacleOption.get()
            pipeVisible= displayPipesOption.get()
            wallDotVisible = displayWallDotsOption.get()
            topDotVisible = displayTopDotsOption.get()
            refreshDisplayObjects(wallVisible, topVisible, obstacleVisible, pipeVisible, topDotVisible, wallDotVisible)

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


        def setCamera(Option, wallShape, topShape):
            if Option == "2DFront":
                camera = 0.5 * wallShape
                return camera
            elif Option == "2DUp":
                wallHeight = comp(wallShape, vector(0, 1, 0))
                wallWidth = comp(wallShape, vector(1, 0, 0))
                topHeight = comp(topShape, vector(0, 1, 0))
                camera = vector(0.5*wallWidth,wallHeight+0.5*topHeight,0)
                return camera

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
            costCounter = 0
            dotCounter = 0
            lengthCounter = 0
            for count, cost in enumerate(object_classes.costDict):
                costCounter += cost

            costCurrentPipes.config(text="P: " +str(round(costCounter,2)) + " €")

            for count, dots in enumerate(object_classes.dotLengthDict):
                dotCounter += dots

            currentDotLength.config(text="C: " +str(dotCounter))

            for count, realLength in enumerate(object_classes.lengthDict):
                lengthCounter += realLength-0.020

            try:
                dotLengthText.text = lengthString + str(dotCounter)
                costText.text = costString + str(round(costCounter,2))
                partText.text = partString + str(pipeCounter)
            except: Exception
            currentRealLength.config(text="actual Length: " +str(round(lengthCounter,3)) + " meter")

            #root.update()




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
        style.configure("Res.TLabel", font=("Calibri", 12), justify="left", width=30)
        style.configure("TEntry", font = ("Calibri", 12), justify="center")
        style.configure("Res.TEntry", font = ("Calibri", 12), justify="center", width =10)
        style.configure("TFrame" , font = ("Calibri", 12), justify="center")


        #create button
        CreateSceneButton=ttk.Button(root, text="Create Scene", command=createNewScene)
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

        displayWallDotsCheckButton=ttk.Checkbutton(root, text= "Display wallDots", variable = displayWallDotsOption)
        displayWallDotsCheckButton.grid(row=5, column=0)

        displayTopDotsCheckButton=ttk.Checkbutton(root, text= "Display TopDots", variable = displayTopDotsOption)
        displayTopDotsCheckButton.grid(row=6, column=0)

        refreshObjectsButton=ttk.Button(root, text="Refresh Objects", command=refreshDisplayObjectsPrep)
        refreshObjectsButton.grid(row=7, column=0)

        topAndWallxSizeLabel=ttk.Label(root, text = "Width:")
        topAndWallxSizeLabel.grid(row=8,column=0)

        topAndWallxSizeEntry = ttk.Entry(root, textvariable = topAndWallxSizeString)
        topAndWallxSizeEntry.grid(row =9, column = 0)

        topHeightLabel=ttk.Label(root, text = "TopHeight:")
        topHeightLabel.grid(row=10,column=0)

        topHeightEntry = ttk.Entry(root, textvariable = topHeightString)
        topHeightEntry.grid(row=11, column = 0)

        wallHeightLabel=ttk.Label(root, text = "wallHeight:")
        wallHeightLabel.grid(row=12,column=0)

        wallHeightEntry = ttk.Entry(root, textvariable = wallHeightString)
        wallHeightEntry.grid(row=13, column = 0)

        defaultParametersButton = ttk.Checkbutton(root, text="Set default Values", command=setDefaultObjectParameters, variable = defaultOption)
        defaultParametersButton.grid(row=14, column=0)

        # part parameters

        pipePartsFrame = ttk.Frame(root)
        pipePartsFrame.place(x=0,y=330)
        SinglePipePartsLengthLabel=ttk.Label(pipePartsFrame, text = "Pipe Length:")
        SinglePipePartsLengthLabel.grid(row=0,column=0)

        SinglePipePartsCountLabel=ttk.Label(pipePartsFrame, text = "Pipe Count:")
        SinglePipePartsCountLabel.grid(row=0,column=1)

        pipePart1LengthEntry = ttk.Entry(pipePartsFrame, textvariable = pl1)
        pipePart1LengthEntry.grid(row=1, column = 0)
        pipePart1CountEntry = ttk.Entry(pipePartsFrame, textvariable=pc1)
        pipePart1CountEntry.grid(row=1, column=1)

        pipePart2LengthEntry = ttk.Entry(pipePartsFrame, textvariable = pl2)
        pipePart2LengthEntry.grid(row=2, column = 0)
        pipePart2CountEntry = ttk.Entry(pipePartsFrame, textvariable=pc2)
        pipePart2CountEntry.grid(row=2, column=1)

        pipePart3LengthEntry = ttk.Entry(pipePartsFrame, textvariable = pl3)
        pipePart3LengthEntry.grid(row=3, column = 0)
        pipePart3CountEntry = ttk.Entry(pipePartsFrame, textvariable=pc3)
        pipePart3CountEntry.grid(row=3, column=1)

        pipePart4LengthEntry = ttk.Entry(pipePartsFrame, textvariable = pl4)
        pipePart4LengthEntry.grid(row=4, column = 0)
        pipePart4CountEntry = ttk.Entry(pipePartsFrame, textvariable=pc4)
        pipePart4CountEntry.grid(row=4, column=1)

        pipePart5LengthEntry = ttk.Entry(pipePartsFrame, textvariable = pl5)
        pipePart5LengthEntry.grid(row=5, column = 0)
        pipePart5CountEntry = ttk.Entry(pipePartsFrame, textvariable=pc5)
        pipePart5CountEntry.grid(row=5, column=1)

        #camera field
        CameraLabel=ttk.Label(root, text = "Camera Parameters:")
        CameraLabel.grid(row=0,column=1)

        TwoDFrontCamOption=ttk.Radiobutton(root, text ="2D Front View", variable = CameraOption, value= "2DFront")
        TwoDFrontCamOption.grid(row=1,column=1)

        TwoDUpViewOption=ttk.Radiobutton(root, text ="2D Up View", variable = CameraOption, value= "2DUp")
        TwoDUpViewOption.grid(row=2,column=1)

        #scene field
        sceneLabel= ttk.Label(root, text = "Scene Parameters:")
        sceneLabel.grid(row=0,column=2)

        backgroundCombobox = ttk.Combobox(root, values=["black", "white"], state="readonly")
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
        parameterOutputFrame.place(x=320,y=250)

        parameterOutputLabel = ttk.Label(parameterOutputFrame, text="Parameter Output:", style = "Res.TLabel")
        parameterOutputLabel.grid(row=0,column=0)

        xDotShowLabel = ttk.Label(parameterOutputFrame,text="xDots: ", style = "Res.TLabel")
        xDotShowLabel.grid(row=1, column=0)

        yDotShowLabel = ttk.Label(parameterOutputFrame,text="yDots: ", style = "Res.TLabel")
        yDotShowLabel.grid(row=2, column=0)

        shiftAtShowLabel = ttk.Label(parameterOutputFrame,text="shift at: ", style = "Res.TLabel")
        shiftAtShowLabel.grid(row=3, column=0)

        currentDotLength = ttk.Label(parameterOutputFrame,text="C: ", style = "Res.TLabel")
        currentDotLength.grid(row=4, column=0)

        costCurrentPipes = ttk.Label(parameterOutputFrame,text="P: ", style = "Res.TLabel")
        costCurrentPipes.grid(row=5, column=0)

        currentRealLength = ttk.Label(parameterOutputFrame,text="current Length: ", style = "Res.TLabel")
        #currentRealLength.grid(row=6, column=0)

        MinOLabel = ttk.Label(parameterOutputFrame,text="MinO: ", style = "Res.TLabel")
        MinOLabel.grid(row=7, column=0)

        priorityWeightLabel = ttk.Label(parameterOutputFrame, text="weights: ")
        priorityWeightLabel.grid(row=0, column=2)

        WeightCLabel = ttk.Label(parameterOutputFrame, text = "Weight C:")
        WeightCLabel.grid(row=1, column = 2)

        WeightPLabel = ttk.Label(parameterOutputFrame,text = "Weight P:")
        WeightPLabel.grid(row=2, column = 2)

        WeightMinOLabel = ttk.Label(parameterOutputFrame,text = "Weight MinO:")
        WeightMinOLabel.grid(row=3, column = 2)

        WeightCEntry = ttk.Entry(parameterOutputFrame, textvariable = gCoption)
        WeightCEntry.grid(row=1, column = 3)

        WeightPEntry = ttk.Entry(parameterOutputFrame, textvariable = gPoption)
        WeightPEntry.grid(row=2, column = 3)

        WeightMinOEntry = ttk.Entry(parameterOutputFrame, textvariable = gMinOoption)
        WeightMinOEntry.grid(row=3, column = 3)

        searchTypeLabel = ttk.Label(parameterOutputFrame, text="search type: ")
        searchTypeLabel.grid(row=0, column=1)

        searchTypeCombobox = ttk.Combobox(parameterOutputFrame, values=["multicriteria astar", "astar", "dijkstra", "best-first"], state="readonly")
        searchTypeCombobox.grid(row=1, column=1)

        def checkSearchType(a):
            if searchTypeCombobox.get()!= "multicriteria astar":
                WeightCEntry.config(state="disabled")
                WeightPEntry.config(state="disabled")
                WeightMinOEntry.config(state="disabled")
            else:
                WeightCEntry.config(state="enabled")
                WeightPEntry.config(state="enabled")
                WeightMinOEntry.config(state="enabled")

        searchTypeCombobox.bind("<<ComboboxSelected>>", checkSearchType)

        refreshPathButton=ttk.Button(parameterOutputFrame, text="Refresh Path", command=refreshPath)
        refreshPathButton.grid(row=3, column=1)

        #LevelSelect

        levelSelectLabel= ttk.Label(root, text = "Level Select:")
        levelSelectLabel.grid(row=0,column=4)

        levelCombobox = ttk.Combobox(root, values=["Level 1 (Easy)", "Level 2 (Medium)", "Level 3 (Hard)", "High Complexity", "Save 1", "Save 2", "Save 3", "Save 4", "Save 5"], state="readonly")
        levelCombobox.grid(row=1, column=4)

        levelCombobox.bind("<<ComboboxSelected>>", disableRefreshPathButton)

        randomlevelSelectLabel= ttk.Label(root, text = "Random Level Creator: ")
        randomlevelSelectLabel.grid(row=2,column=4)

        randomizeCheckButton=ttk.Checkbutton(root, text= "Create random Level", variable =randomizeOption, command = setRandomizeLevel, style = "Nowidth.TCheckbutton")
        randomizeCheckButton.grid(row=3,column=4)

        obstacleProbabilityLabel= ttk.Label(root, text = "Obstacle frequency: ")
        obstacleProbabilityLabel.grid(row=4,column=4)

        obstacleProbabilityCombobox = ttk.Combobox(root, values=["0: Empty", "1: Very Low", "2: Low", "3: Medium", "4: High", "5: Very High", "6: Extreme", "7: Very Extreme", "8: Ultimate"], state = "readonly")
        obstacleProbabilityCombobox.grid(row=5, column=4)


        # debugField
        debugOptionsLabel= ttk.Label(root, text = "Showcase options:")
        debugOptionsLabel.grid(row=0,column=5)

        showTestingPathsCheckButton=ttk.Checkbutton(root, text= "Show path finding", variable = showTestingPathsOption, style = "Nowidth.TCheckbutton")
        showTestingPathsCheckButton.grid(row=1,column=5)

        showTestedPathsCheckButton=ttk.Checkbutton(root, text= "Show tested positions", variable = showTestedPathsOption, style = "Nowidth.TCheckbutton")
        showTestedPathsCheckButton.grid(row=2,column=5)

        #insert default values
        topAndWallxSizeEntry.insert(0, "100.0")
        topHeightEntry.insert(0, "115.0")
        wallHeightEntry.insert(0, "200.0")
        pipePart1LengthEntry.insert(0,"1")
        pipePart1CountEntry.insert(0,"10")
        pipePart2LengthEntry.insert(0,"2")
        pipePart2CountEntry.insert(0,"10")
        pipePart3LengthEntry.insert(0,"3")
        pipePart3CountEntry.insert(0,"10")
        pipePart4LengthEntry.insert(0,"4")
        pipePart4CountEntry.insert(0,"10")
        pipePart5LengthEntry.insert(0,"5")
        pipePart5CountEntry.insert(0,"10")
        WeightCEntry.insert(0,"1")
        WeightPEntry.insert(0,"1")
        WeightMinOEntry.insert(0,"1")
        backgroundCombobox.set("white")
        searchTypeCombobox.set("multicriteria astar")
        levelCombobox.set("Level 1 (Easy)")
        TwoDFrontCamOption.invoke()
        resolutionEntryW.insert(0, GetSystemMetrics(0))
        resolutionEntryH.insert(0, GetSystemMetrics(1))
        displayPipesCheckButton.invoke()
        displayTopCheckButton.invoke()
        displayWallCheckButton.invoke()
        displayObstacleCheckButton.invoke()
        displayWallDotsCheckButton.invoke()
        displayTopDotsCheckButton.invoke()
        obstacleProbabilityCombobox.config(state="disabled")
        obstacleProbabilityCombobox.set("0: Empty")
        refreshObjectsButton.config(state="disabled")
        displayPipesCheckButton.config(state="disabled")
        displayTopCheckButton.config(state="disabled")
        displayWallCheckButton.config(state="disabled")
        displayObstacleCheckButton.config(state="disabled")
        displayWallDotsCheckButton.config(state="disabled")
        displayTopDotsCheckButton.config(state="disabled")
        refreshPathButton.config(state="disabled")

        #call functions that check after some time
        calcDotCall()


        root.mainloop()





    #functions

def create_Route(xDots, yDots, start, end, wallToTopShiftDots, startAxis, goalAxis,testingPath,testedPath, heuristicType, pipeTypeDict, search_type ,gC,gP,gMinO):
    route, parts = agt.displayPlot_Call(xDots, yDots, start, end, wallToTopShiftDots, startAxis, goalAxis,testingPath,testedPath, heuristicType, pipeTypeDict, search_type,gC,gP,gMinO)
    if isinstance(route, list):
        for idx,(x,y) in enumerate(route):
             route[idx] = (x+1,y+1)
        return route, parts
    else: return False, False

def pipeTypeDictEmpty(dict):
    for index, (key, count) in enumerate(dict.items()):
        empty = False
        if count < 0:
            empty = True
            return empty
    return empty


def determineType(part):

    if part == 5:
         type="red"
    elif part == 4:
          type="yellow"
    elif part == 3:
         type="blue"
    elif part == 2:
        type ="green"
    elif part == 1:
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




def plotGraph(search_type, shiftpos, route, start, goal, M):
            if search_type == "multicriteria astar":
                search_type = "MCA*"
                col = "blue"
            elif search_type == "astar":
                search_type = "A*"
                col = "cyan"
            elif search_type == "best-first":
                search_type = "Bestensuche"
                col = "yellow"
            else:
                search_type = "Dijkstra"
                col = "orange"
            x_coords = []
            y_coords = []
            x_acoords = []
            y_acoords = []
            shiftcoordsX = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
            shiftcoordsY = [shiftpos - 1, shiftpos - 1, shiftpos - 1, shiftpos - 1, shiftpos - 1, shiftpos - 1,
                            shiftpos - 1, shiftpos - 1, shiftpos - 1, shiftpos - 1]

            for i in (range(0, len(route))):
                x = route[i][0] -1
                y = route[i][1] -1
                x_coords.append(x)
                y_coords.append(y)

            aroute = [(10, 1), (7, 1), (7, 5), (9, 5), (9, 11), (6, 11), (6, 16), (8, 16), (8, 21), (2, 21), (2, 23), (1, 23)]

            for i in (range(0, len(aroute))):
                x = aroute[i][0] -1
                y = aroute[i][1] -1
                x_acoords.append(x)
                y_acoords.append(y)

            # plot map and path
            font = {'family': 'normal',
                    'weight': 'bold',
                    'size': 22}
            plt.rc('font', **font)
            fig, ax = plt.subplots(figsize=(20, 20))
            ax.imshow(M, cmap=plt.cm.Greys)
            ax.scatter(start[1]-1, start[0]-1, marker="*", color="green", s=600, label = "Start")
            ax.scatter(goal[1]-1, goal[0]-1, marker="*", color="red", s=600, label = "Ziel")
            plt.xlabel("Y-Koordinate")
            plt.ylabel("X-Koordinate")
            #plt.legend(loc="upper left")
            ax.plot(y_coords, x_coords, color=col, label = search_type, linewidth=2.5)
            ax.plot(y_acoords, x_acoords, color="cyan", label="A*", linewidth=2.5)
            ax.plot(shiftcoordsY, shiftcoordsX, color="red", label = "Übergang")
            plt.legend()
            plt.show()

def buildVpipes(buildPipeDict, buildClampDict):
    tVisual = open("visualRefreshTime.txt", "a")
    startTime = datetime.now()
    for count, objects in enumerate(buildPipeDict):
        part = objects[0]
        if part == "red":
            newPipe = object_classes.red
        elif part == "yellow":
            newPipe = object_classes.yellow
        elif part == "blue":
            newPipe = object_classes.blue
        elif part == "green":
            newPipe = object_classes.green
        elif part == "purple":
            newPipe = object_classes.purple
        else:
            newPipe = object_classes.corner
        newPipe.showObject(objects[1], objects[2])
        newPipe.countObject()
    for count, objects in enumerate(buildClampDict):
        object_classes.clamp(objects[0], objects[1], True)
    execTime = datetime.now() - startTime
    tVisual.write(str(execTime.total_seconds()) + "\n")


def pipeBuilder(cRoute, parts, pipeVisible, start, startAxis, goal, goalAxis, wallToTopShiftDots, wallVisible, topVisible, pipeTypeDict):
    global pipeCounter
    global MinOValue
    MinOValue = 0
    pipeBuildDict = []
    clampDict = []
    pipeCounter = deepcopy(pipeTypeDict)
    Matrix = lvf.get_boolGrid()
    if cRoute == False:
        return
    for idx, (x,y) in enumerate(cRoute):

        #dont check next point if last point has been reached
        if idx == len(cRoute)-1:
            break

        #set pointA and pointB, calculate difference
        pointA=cRoute[idx]
        pointB=cRoute[idx+1]
        differenceX = list(pointB)[0] - list(pointA)[0]
        differenceY = list(pointB)[1] - list(pointA)[1]
        diff = (differenceX, differenceY)
        MinOValue = MinOValue + agt.costMinO(Matrix,(pointA[0]-1,pointA[1]-1),diff) * abs(diff[0] - diff[1])*2

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
        partValue = parts[(pointB[0]-1,pointB[1]-1)]
        type = determineType(partValue)
        pipeCounter[partValue] = pipeCounter[partValue] - 1

        if pointB == goal and pointA[1] == goal[1]:
            nearGoalHor = True
        elif pointB == goal and pointA[0] == goal[0]:
            nearGoalVert = True
        else:
            nearGoalHor = False
            nearGoalVert = False

        #create pipe with determined type and axis, create corners corresponding to pipe

        #check if we are at shiftpoint
        atShiftPoint = False
        if pointA == (x, wallToTopShiftDots):
            atShiftPoint = True
        else: atShiftPoint = False
        #todo: create a function for pipe creation
        if atShiftPoint == True:
            if axis == lvf.up and pointB != goal:
                pipe_coords = (x + add[0], y + add[1] + 1)
                corner_coords = (x, y+1)
                corner_axis = lvf.determineCorner(previousAxis, axis)

                #Objects.pipe(type, pipe_coords, axis, pipeVisible)
                #corner = Objects.pipe("corner", corner_coords, corner_axis, pipeVisible)

                pipeBuildDict.append((type, pipe_coords, axis, pipeVisible))
                pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))
                # if corner_axis == lvf.totop:
                #     corner.corner.rotate(angle=-0.5 * pi)  # rotate object
            elif pointB == goal:
                closeWithoutCorner = True
                if goalAxis==lvf.up and nearGoalHor == True:
                    closeWithoutCorner = False
                elif goalAxis==lvf.down and nearGoalHor == True:
                    closeWithoutCorner = False
                elif goalAxis == lvf.right and nearGoalVert == True:
                    closeWithoutCorner = False
                elif goalAxis == lvf.left and nearGoalVert == True:
                    closeWithoutCorner = False

                if closeWithoutCorner == True:

                    pipe_coords = (x + add[0], y + add[1] + 1)
                    corner_coords = (x, y+1)
                    corner_axis = lvf.determineCorner(previousAxis, axis)

                    #Objects.pipe(type, pipe_coords, axis, pipeVisible)
                    #corner = Objects.pipe("corner", corner_coords, corner_axis, pipeVisible)

                    pipeBuildDict.append((type, pipe_coords, axis, pipeVisible))
                    pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))

                    # if corner_axis == lvf.totop:
                    #     corner.corner.rotate(angle=-0.5 * pi)  # rotate object
                else:

                    pipe_coords = (x + add[0], y + add[1] + 1)
                    corner_coords = (x, y + 1)
                    corner_end_coords = (x + differenceX, y + differenceY)
                    corner_axis = lvf.determineCorner(previousAxis, axis)
                    corner_axis_end = lvf.determineCorner(axis, -goalAxis)

                    #Objects.pipe(type, pipe_coords, axis, pipeVisible)
                    #corner = Objects.pipe("corner", corner_coords, corner_axis, pipeVisible)
                    #cornerEnd = Objects.pipe("corner", corner_end_coords, corner_axis_end, pipeVisible)

                    pipeBuildDict.append((type, pipe_coords, axis, pipeVisible))
                    pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))
                    pipeBuildDict.append(("corner", corner_end_coords, corner_axis_end, pipeVisible))

                    # if corner_axis == lvf.totop:
                    #     corner.corner.rotate(angle=-0.5 * pi)  # rotate object
            else:
                pipe_coords = (x + add[0], y + add[1])
                corner_coords = (x, y)
                corner_axis = lvf.determineCorner(previousAxis, axis)

                #Objects.pipe(type, pipe_coords, axis, pipeVisible)
                #corner = Objects.pipe("corner", corner_coords, corner_axis, pipeVisible)

                pipeBuildDict.append((type, pipe_coords, axis, pipeVisible))
                pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))

                # if corner_axis == lvf.totop:
                #     corner.corner.rotate(angle=-0.5 * pi)  # rotate object

        else:
            if pointA == start:
                add, ax = determineAxis(diff)
                if ax != startAxis:
                    if ax == lvf.right:
                        pipe_coords = (x+add[0],y+add[1])
                        corner_coords = (x, y)
                        corner_axis = lvf.determineCorner(startAxis, axis)
                        pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))
                    elif ax == lvf.left:
                        pipe_coords = (x+add[0],y+add[1])
                        corner_coords = (x, y)
                        corner_axis = lvf.determineCorner(startAxis, axis)
                        pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))
                    elif ax == lvf.up:
                        pipe_coords = (x+add[0],y+add[1])
                        corner_coords = (x, y)
                        corner_axis = lvf.determineCorner(startAxis, axis)
                        pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))
                    else:
                        pipe_coords = (x,y-1+add[1])
                        corner_coords = (x, y)
                        corner_axis = lvf.determineCorner(startAxis, axis)
                        pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))
                else:
                    pipe_coords = (x,y)

                pipeBuildDict.append((type, pipe_coords, axis, pipeVisible))

            elif nearGoalHor == True and goalAxis==lvf.up or nearGoalHor == True and goalAxis==lvf.down or \
                    nearGoalVert == True and goalAxis == lvf.right or nearGoalVert == True and goalAxis == lvf.left:

                pipe_coords = (x+add[0],y+add[1])
                corner_coords = (x,y)
                corner_axis= lvf.determineCorner(previousAxis, axis)
                corner_end_coords = (x+differenceX,y+differenceY)
                corner_axis_end = lvf.determineCorner(axis, -goalAxis)

                #Objects.pipe(type, pipe_coords, axis, pipeVisible)
                #corner = Objects.pipe("corner", corner_coords, corner_axis, pipeVisible)
                #cornerEnd = Objects.pipe("corner", corner_end_coords, corner_axis_end, pipeVisible)

                pipeBuildDict.append((type, pipe_coords, axis, pipeVisible))
                pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))
                pipeBuildDict.append(("corner", corner_end_coords, corner_axis_end, pipeVisible))

                # if corner_axis == lvf.totop:
                #     corner.corner.rotate(angle=-0.5 * pi)  # rotate object
            else:
                pipe_coords = (x+add[0],y+add[1])
                corner_coords = (x,y)
                corner_axis= lvf.determineCorner(previousAxis, axis)

                #Objects.pipe(type, pipe_coords, axis, pipeVisible)
                #corner = Objects.pipe("corner", corner_coords, corner_axis, pipeVisible)

                pipeBuildDict.append((type, pipe_coords, axis, pipeVisible))
                pipeBuildDict.append(("corner", corner_coords, corner_axis, pipeVisible))


                # if corner_axis == lvf.totop:
                #     corner.corner.rotate(angle=-0.5 * pi)  # rotate object
        clampCoord = lvf.determineClampPlacement(Matrix, pipe_coords, diff, partValue)
        clampDict.append((clampCoord, axis))
    buildVpipes(pipeBuildDict, clampDict)



def RandomLevelCreator(frequency, wallToTopShiftDots, xDots, yDots, obsVisWall, obsVisTop):
    frequencyWall = frequency
    frequencyTop = int(frequency*0.6)
    checkList = []
    #create random Object on wall
    i = 0
    while i < frequencyWall:
        #if random.random() <= probability:
        randPosX = random.randint(1, xDots)
        randPosY = random.randint(1, wallToTopShiftDots)
        randSizeX = 1
        randSizeY = 1
        #else: continue
        if (randPosX,randPosY) in checkList:
            continue
        else:
            object_classes.obstacle((randSizeX, randSizeY), (randPosX, randPosY), obsVisWall)
            checkList.append((randPosX,randPosY))
            i=i+1
    #create random Objects on Top
    i = 0
    while i < frequencyTop:
        randPosX = random.randint(1, xDots)
        randPosY = random.randint(wallToTopShiftDots+2, yDots)
        randSizeX = 1
        randSizeY = 1
        if (randPosX,randPosY) in checkList:
            continue
        else:
            object_classes.obstacle((randSizeX, randSizeY), (randPosX, randPosY), obsVisTop)
            checkList.append((randPosX,randPosY))
            i=i+1

def createScene(wallShape, topShape, wallThickness, wallcolor, topcolor, dotcolor, lampvisible, wallVisible, topVisible,
            obstacleVisible, pipeVisible, topDotVisible, wallDotVisible, coordinateInfoVisible, camera, backgroundColor, xDots, yDots, xGap, yGap,
            dotDist, wallToTopShiftDots,testingPath,testedPath,level,xRes, yRes, heuristicType, refresh, pipeTypeDict, search_type,gC,gP,gMinO):
    #create wall
    if refresh == False:

        execTime = datetime.now()
        object_classes.PipeLabInstance(wallShape, topShape, wallThickness, wallcolor, topcolor, lampvisible, wallVisible, topVisible,
                                       coordinateInfoVisible, camera, backgroundColor, xRes, yRes)
        # create logic matrix
        lvf.cdCm_Call(x_dots=xDots, y_dots=yDots, x_gap=xGap, y_gap=yGap, dot_dist=dotDist, wall_thickness=wallThickness,
                      dot_color=dotcolor, wall_to_top_shift_dots=wallToTopShiftDots, top_visible=topVisible, wall_visible=wallVisible)

    #obstacle chronology go from bottom to top, left to right
    if wallVisible == True and obstacleVisible == True:
        obsVisWall = True
    else: obsVisWall = False

    if topVisible == True and obstacleVisible == True:
        obsVisTop = True
    else: obsVisTop = False

    random = False
    object_classes.costDict.clear()
    object_classes.lengthDict.clear()
    object_classes.dotLengthDict.clear()

    try:
        for key in object_classes.pipeDict.keys():
            obstDel = object_classes.obstacleDict[key]
            del obstDel
    except: Exception

    if refresh == False:
        levelInfo = levelData.selectLevel(level, backgroundColor, wallVisible, topVisible, obsVisWall, obsVisTop)
        if not levelInfo:
            random = True
            frequency = levelData.selectRandomLevel(level)
            levelInfo = levelData.randomPrepInit(xDots, yDots, backgroundColor, wallVisible, topVisible)
        start = levelInfo[0]
        goal = levelInfo[1]
        startAxis = levelInfo[2]
        goalAxis = levelInfo[3]

    cMatrix_route = ""
    if refresh == True:
        for key in object_classes.pipeDict.keys():
            oldPipe = object_classes.pipeDict[key]
            oldPipe.color = color.gray(0.3)
            #oldPipe.opacity = 0.5
            del oldPipe
        for key in object_classes.showcaseDict.keys():
            sBox = object_classes.showcaseDict[key]
            sBox.visible = False
            del sBox

        start = object_classes.savedState[0]
        goal = object_classes.savedState[1]
        startAxis = object_classes.savedState[2]
        goalAxis = object_classes.savedState[3]

        cMatrix_route, parts = create_Route(xDots, yDots, start, goal, wallToTopShiftDots, startAxis, goalAxis,
                                     testingPath,
                                     testedPath, heuristicType, pipeTypeDict, search_type,gC,gP,gMinO)
        print(cMatrix_route)
        if pipeVisible == True:
            if isinstance(cMatrix_route, list):
                pipeBuilder(cMatrix_route, parts, pipeVisible, start, startAxis, goal, goalAxis, wallToTopShiftDots,
                            wallVisible,
                            topVisible, pipeTypeDict)
        return

    if random == False:
        if pipeVisible == True:
            cMatrix_route, parts = create_Route(xDots, yDots, start, goal, wallToTopShiftDots, startAxis, goalAxis,
                                                testingPath,
                                                testedPath, heuristicType, pipeTypeDict, search_type,gC,gP,gMinO)
            print(cMatrix_route)
            if isinstance(cMatrix_route, list):
                plotGraph(search_type, wallToTopShiftDots, cMatrix_route, start, goal, lvf.get_boolGrid())
                pipeBuilder(cMatrix_route, parts, pipeVisible, start,startAxis, goal, goalAxis, wallToTopShiftDots, wallVisible, topVisible, pipeTypeDict)
    elif random == True:
        # while isinstance(cMatrix_route, list) == False:
        #     Objects.resetObstacles()
        #     Objects.resetShowcase()
        #
        #     Objects.PipeLabInstance(wallShape, topShape, wallThickness, wallcolor, topcolor, lampvisible, wallVisible,
        #                          topVisible,
        #                             coordinateInfoVisible, camera, backgroundColor, xRes, yRes)
        lvf.cdCm_Call(x_dots=xDots, y_dots=yDots, x_gap=xGap, y_gap=yGap, dot_dist=dotDist,
                      wall_thickness=wallThickness,
                      dot_color=dotcolor, wall_to_top_shift_dots=wallToTopShiftDots, top_visible=topVisible,
                      wall_visible=wallVisible)
        RandomLevelCreator(frequency, wallToTopShiftDots, xDots, yDots, obsVisWall, obsVisTop)
        try:
            vIT= open("visualInitTime.txt", "a")
            execTime = datetime.now() - execTime
            vIT.write(str(execTime.total_seconds()) + "\n")

        except: Exception
        cMatrix_route, parts = create_Route(xDots, yDots, start, goal, wallToTopShiftDots, startAxis, goalAxis,
                                     testingPath,
                                     testedPath, heuristicType, pipeTypeDict, search_type,gC,gP,gMinO)
        print(cMatrix_route)
        if pipeVisible == True:
            plotGraph(search_type, wallToTopShiftDots, cMatrix_route, start, goal, lvf.get_boolGrid())
            pipeBuilder(cMatrix_route, parts, pipeVisible, start, startAxis, goal, goalAxis, wallToTopShiftDots, wallVisible,
                    topVisible, pipeTypeDict)



def refreshObjects(visible, dict):
    if visible == True:
        for key in dict.keys():
            object = dict[key]
            object.visible = True
    else:
        for key in dict.keys():
            object = dict[key]
            object.visible = False



def refreshDisplayObjects(wallVisible, topVisible, obstacleVisible, pipeVisible, topDotVisible, wallDotVisible):
    refreshObjects(wallVisible, object_classes.wallDict)
    refreshObjects(topVisible, object_classes.topDict)
    refreshObjects(pipeVisible, object_classes.pipeDict)
    refreshObjects(obstacleVisible, object_classes.obstacleDict)
    refreshObjects(wallDotVisible, lvf.wallDotDict)
    refreshObjects(topDotVisible, lvf.topDotDict)


#start the app
if __name__ == "__main__":
    scene = canvas()
    app = App()



