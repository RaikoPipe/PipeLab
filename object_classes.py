from vpython import *
import LogicalVfunctions as lvf
import main
import weakref

'All Wall Objects are relative to Vector(0,0,0)'
'All other objects are relative to dotCoordMatrix(0,0)'


global obstacleDict
obstacleDict = weakref.WeakValueDictionary()
global pipeDict
pipeDict = weakref.WeakValueDictionary()
global wallDict
wallDict = weakref.WeakValueDictionary()
global topDict
topDict = weakref.WeakValueDictionary()
global showcaseDict
showcaseDict = weakref.WeakValueDictionary()
global costDict
costDict = []
global dotLengthDict
dotLengthDict = []
global lengthDict
lengthDict = []
global savedState
savedState = []

# this function creates the canvas, wall and top and dots

def resetShowcase():
    for key in showcaseDict.keys():
        sBox = showcaseDict[key]
        sBox.visible = False

def resetObstacles():
    for key in obstacleDict.keys():
        oldObs = obstacleDict[key]
        oldObs.visible = False
        del oldObs



def PipeLabInstance(wall_shape, top_shape, wall_thickness, wall_color, top_color, lamp_visible,
             wall_visible, top_visible, coordinate_info_visible, camera_pos, background_color,x_res,y_res):

    main.scene.delete()
    del main.scene

    # creating the scene
    main.scene = canvas(width=x_res, height=y_res, center=camera_pos, background=background_color, fov=pi/5)
    main.scene.camera.rotate(angle=pi/2, axis=vector(0,0,1))
    print("Detected resolution: " + str(x_res) + "x" + str(y_res))


    # calculating some vectors needed for Projection, in case wall changes size
    z_vector = vector(0, 0, 1)  # needed for projection
    x_vector = vector(1, 0, 0)  # needed for projection

    # initialise top, set position for top_shape relative to its own size and wall x and z coordinate,
    wallHeight = comp(wall_shape, vector(0,1,0))
    wallWidth = comp(wall_shape, vector(1,0,0))
    topHeight=comp(top_shape, vector(0,1,0))
    top_pos = vector(0.5*wallWidth,wallHeight+0.5*topHeight,0.5*wall_thickness)

    top_shape_x_pos = (proj(top_shape, z_vector))  # gets vector with z coordinate only
    top_shape_z_pos = (proj(wall_shape, x_vector))  # gets vector with x coordinate only
    # top_pos = wall_shape - 0.5 * top_shape_z_pos+ 0.5 * top_shape_x_pos #calculate top position

    topwrap = box(pos=top_pos, size=top_shape, color=vector(0.689, 0.515, 0.412), shininess=0.0)  # create top Object
    top = box(pos = top_pos + (proj(top_pos, vector(0,0,1))) + vector(0,0,0.1), size =  top_shape - (proj(top_shape, vector(0,0,1)) + vector(0,0,0.1)),
                    color = top_color, shininess =0.0)
    lvf.remember(topwrap, topDict)
    lvf.remember(top, topDict)
    pass



    # initialise wall, set position, set size

    wallwrap = box(pos=0.5 * wall_shape, size=wall_shape
                    , color=vector(0.689, 0.515, 0.412),
                    shininess=0.0)  # size needs to be a vector(x,y,z) (vector(width, height, depth))
    wall = box(pos = 0.5 * wall_shape + 0.5*(proj(wall_shape, vector(0,0,1))) + vector(0,0,0.1), size =  wall_shape - (proj(wall_shape, vector(0,0,1)) + vector(0,0,0.1)),
                    color = wall_color, shininess =0.0)
    lvf.remember(wallwrap, wallDict)
    lvf.remember(wall, wallDict)
    pass

    # setlights
    lamp_wall = local_light(pos=0.5*wall_shape, color=color.white)  # set a light in front of the wall
    lampsphere_wall = sphere(pos=0.5*wall_shape, color=color.yellow, emissive=True,
                             visible=lamp_visible)  # set an emissive sphere on light pos.
    lamp_top = local_light(pos=0.5*top_pos, color=color.white)  # set a light in front of the wall
    lampsphere_top = sphere(pos=0.5*top_pos, color=color.yellow, emissive=True,
                             visible=lamp_visible)  # set an emissive sphere on light pos.

    #setdivider
    if wall_visible and top_visible:
        dividerPos = vector(0.5*wallWidth, wallHeight,0.5*wall_thickness)
        divider = box(pos=dividerPos, size = vector(wallWidth,2,20), color=color.red)

    if background_color == color.black:
        text_color = color.white
    else: text_color = color.black

    # initialise labeled Coordinate points; helps with orientation
    if coordinate_info_visible == True:
        vector_0x0_point = points(pos=vector(0.0, 0.0, wall_thickness), color=color.green)
        vector_100x200 = points(pos=wall_shape, color=color.green)
        vector_maxwidthheight = points(pos=vector(wallWidth,wallHeight+topHeight,wall_thickness), color=color.green)
        vector_0x0_point_label = label(pos=vector(0.0, 0.0, wall_thickness), text="0cm x 0cm", xoffset=-20, yoffset=-50,
                                       space=30, height=16, border=4, font="sans", color=text_color)
        vector_100x200_point_label = label(pos=wall_shape, text=str(wallWidth) + "cm" + " x " +str(wallHeight)+"cm", xoffset=20, yoffset=50, space=30,
                                           height=16, border=4, font="sans", color=text_color)
        vector_0x0_point_label = label(pos=vector(0.0, 0.0, wall_thickness), text="0cm x 0cm", xoffset=-20, yoffset=-50,
                                       space=30, height=16, border=4, font="sans", color=text_color)
        vector_maxwidthheight_label = label(pos=vector(wallWidth,wallHeight+topHeight,wall_thickness), text=str(wallWidth) + "cm" + " x " +str(wallHeight+topHeight)+"cm", xoffset=20, yoffset=50, space=30,
                                           height=16, border=4, font="sans", color=text_color)

    #initialise corner object
    CornerPipeTemp = extrusion(path=paths.arc(radius=2.6 * 3, angle1=-0.09, angle2=pi / 2 + 0.092),
                               shape=shapes.arc(radius=2.6, angle1=0, angle2=2 * pi, rotate=pi),
                               color=color.cyan, visible = False)
    global cornerTemp
    cornerTemp = compound([CornerPipeTemp], up=vec(0, 0, 1), visible = False)

    #initialise SinglePipe Objects
    global SinglePipeTemp
    SinglePipeTemp = cylinder()








def StartEndInt(start_position, end_position, start_direction, end_direction, background_color, wallvisible, topvisible):
    if background_color == color.black:
        text_color = color.white
    else: text_color = color.black
    # initialise Start and End position Cylinders
    startPos = lvf.cMatrix[start_position]
    endPos = lvf.cMatrix[end_position]


    startcylinder = cylinder(pos=lvf.transformToVvector(startPos) + vector(0, 0, 5) - start_direction,axis=start_direction, size=vector(1, 8, 8),
                             color=color.green, visible = wallvisible)
    startArrow = arrow(pos=lvf.transformToVvector(startPos) + vector(0, 0, 5) - start_direction*5,axis=start_direction, size=vector(20, 8, 8),
                             color=color.green, visible = wallvisible)
    start_label = label(pos=lvf.transformToVvector(startPos)- start_direction*6, text="Start", space=30,
                        height=30, border=4,
                        font="sans", color=text_color, visible = wallvisible, opacity = 1, line = True)
    lvf.remember(startcylinder, wallDict)
    lvf.remember(start_label, wallDict)
    lvf.remember(startArrow, wallDict)

    endcylinder = cylinder(pos=lvf.transformToVvector(endPos) + vector(0, 0, 5) - end_direction, axis=end_direction, size=vector(1, 8, 8),
                           color=color.orange, visible = topvisible)
    endArrow = arrow(pos=lvf.transformToVvector(endPos) + vector(0, 0, 5) - end_direction, axis=-end_direction, size=vector(20, 8, 8),
                           color=color.orange, visible = topvisible)

    end_label = label(pos=lvf.transformToVvector(endPos) - end_direction*6, text="Goal", space=30,
                      height=30, border=4,
                      font="sans", color=text_color, visible= topvisible, opacity = 1, line= True)
    lvf.remember(endcylinder, topDict)
    lvf.remember(end_label, topDict)
    lvf.remember(endArrow,topDict)

    pass

class status_label:
    def __init__(self, name, info):
        newLabel = label(pos=(20,20,20), xoffset = 30, text=name+info, space=30,
                        height=16, border=4,
                        font="sans", color=color.white)



def createSocket(pipe_coord, pipe_axis,socketDotDistance, pipe_visible):
    pipe_length = 8.8
    pipe_width = 5.5
    overhang = 4.5
    dotlength = 1
    socket_pos = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, socketDotDistance)]
    directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, overhang, pipe_width - 0.5)
    # if validPlacement(position, pipe_length) == True
    socket = cylinder(pos=lvf.transformToVvector(socket_pos) + directionalOverhang, axis=pipe_axis,
             size=vector(pipe_length, pipe_width, pipe_width),
             color=color.black, visible=pipe_visible)
    lvf.setOccP_Call(pipe_coord, dotlength, pipe_axis)
    #print("socket created at position: " + str(pipe_coord))
    lvf.remember(socket, pipeDict)

class SinglePipe:
    def __init__(self, pipeLength, dotLength, cost, pipe_visible, color):
        self.pipeLength = pipeLength
        self.realMeter = pipeLength/100
        self.pipeWidth = 5
        self.dotLength = dotLength
        self.overhang = 4.5
        self.color = color
        self.cost = cost
        self.pipeVisible = pipe_visible

    def showObject(self, pipeCoord, pipeAxis ):
        self.pipe_pos = lvf.cMatrix[pipeCoord]
        self.pipe_axis = pipeAxis
        self.directionalOverhang = lvf.determineDirectionalOverhang(type, self.pipe_axis, self.overhang, self.pipeWidth)
        self.pipe = SinglePipeTemp.clone(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang, axis=self.pipe_axis,
                          size=vector(self.pipeLength, self.pipeWidth, self.pipeWidth),
                          color= color, visible=self.pipeVisible)
        lvf.remember(self.pipe, pipeDict)
    def countObject(self):
        costDict.append(self.cost)
        dotLengthDict.append(self.dotLength)
        lengthDict.append(self.realMeter)


purple = SinglePipe(pipeLength=8.8,dotLength=1,cost=1.15, color = color.purple, pipe_visible=True)
green = SinglePipe(pipeLength=19.3,dotLength=2,cost=1.38, color = color.green, pipe_visible=True)
blue = SinglePipe(pipeLength=29.8,dotLength=3,cost=1.6, color = color.blue, pipe_visible=True)
yellow = SinglePipe(pipeLength=40.3,dotLength=4,cost=1.82, color = color.yellow, pipe_visible=True)
red = SinglePipe(pipeLength=50.8,dotLength=5,cost=2.04, color = color.red, pipe_visible=True)


# this class creates all pipe objects
class pipe:
    def __init__(self, type, pipe_coord, pipe_axis, pipe_visible):
        self.pipe_pos = lvf.cMatrix[pipe_coord]
        self.pipe_ax = pipe_axis
        if type == "blue":
            self.pipe_length = 29.8
            self.pipe_width = 5
            self.overhang = 4.5
            self.dotlength = 3
            self.cost = 1.60
            self.realMeter = 0.298
            self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)

            blue = SinglePipe(type, self.pipe_pos, pipe_axis, pipe_visible, color.blue, self.pipe_length,
                                self.pipe_width, self.overhang)

            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)

            #print("blue created at position: " + str(pipe_coord))

            costDict.append(self.cost)
            dotLengthDict.append(self.dotlength)
            lengthDict.append(self.realMeter)

        elif type == "green":
            self.pipe_length = 19.3
            self.pipe_width = 5
            self.overhang = 4.5
            self.dotlength = 2
            self.cost =1.38
            self.realMeter = 0.193
            self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)

            green = SinglePipe(type, self.pipe_pos, pipe_axis, pipe_visible, color.green, self.pipe_length,
                                self.pipe_width, self.overhang)
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)

            costDict.append(self.cost)
            dotLengthDict.append(self.dotlength)
            lengthDict.append(self.realMeter)


        elif type == "purple":
            self.pipe_length = 8.8
            self.pipe_width = 5
            self.overhang = 4.5
            self.dotlength = 1
            self.cost = 1.15
            self.realMeter = 0.088
            self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)

            purple = SinglePipe(type, self.pipe_pos, pipe_axis, pipe_visible, color.purple, self.pipe_length,
                                self.pipe_width, self.overhang)
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)

            costDict.append(self.cost)
            dotLengthDict.append(self.dotlength)
            lengthDict.append(self.realMeter)

        elif type == "yellow":
            self.pipe_length = 40.3
            self.pipe_width = 5
            self.overhang = 4.5
            self.dotlength = 4
            # self.cost = 1.82
            self.cost = 20
            self.realMeter = 0.403
            self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)

            yellow = SinglePipe(type, self.pipe_pos, pipe_axis, pipe_visible, color.yellow, self.pipe_length,
                                self.pipe_width, self.overhang)
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)

            costDict.append(self.cost)
            dotLengthDict.append(self.dotlength)
            lengthDict.append(self.realMeter)

        elif type == "red":
            self.pipe_length = 50.8
            self.pipe_width = 5
            self.overhang = 4.5
            self.dotlength = 5
            self.cost =  2.04
            self.realMeter = 0.403
            self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
            # if validPlacement(position, pipe_length) == True
            red = SinglePipe(type, self.pipe_pos, pipe_axis, pipe_visible, color.red, self.pipe_length,
                                self.pipe_width, self.overhang)



            #print("long4 created at position: " + str(pipe_coord))

            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)

            costDict.append(self.cost)
            dotLengthDict.append(self.dotlength)
            lengthDict.append(self.realMeter)

        elif type == "corner":
            self.dotlength = 1
            self.cost = 5.32
            self.realMeter = 0.082  # 112mm - (diameter of a pipe(50)/2) - thickness(5mm)

            cornerAdd = Corner(type, self.pipe_pos, pipe_axis , pipe_visible)
            lvf.setOccP_Call(pipe_coord, self.dotlength, pipe_axis)

            costDict.append(self.cost)
            dotLengthDict.append(self.dotlength)
            lengthDict.append(self.realMeter)
        else:
            sizeVector= vector(1*10.5, 1*10.5, 5)
            box(size=sizeVector, pos = lvf.transformToVvector(self.pipe_pos), color=color.red, visible=True, opacity = 0.9)
            #lvf.setOccO_Call(1,1, self.pipe_pos)
            print("Error" +" at position: " + str(pipe_coord))




class SinglePipe():
    def __init__(self, type, pipe_pos, pipe_axis, pipe_visible, color, pipe_length, pipe_width, overhang):

        self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, overhang, pipe_width)
        self.pipe = SinglePipeTemp.clone(pos=lvf.transformToVvector(pipe_pos) + self.directionalOverhang, axis=pipe_axis,
                          size=vector(pipe_length, pipe_width, pipe_width),
                          color= color, visible=pipe_visible)

        lvf.remember(self.pipe, pipeDict)



class Corner():
    def __init__(self, type, pipe_pos, pipe_axis, pipe_visible):
        self.pipe_radius = 2.6
        self.overhang = -8


        self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_radius)
        self.corner = cornerTemp.clone(pos = lvf.transformToVvector(pipe_pos) + self.directionalOverhang,
                                       axis = pipe_axis, visible = pipe_visible)

        if pipe_axis == lvf.totop:
            self.corner.rotate(angle=-0.5 * pi)


        lvf.remember(self.corner, pipeDict)


# create obstacles
class obstacle:
    def __init__(self, size, position, obstacle_visible):
        size_x = size[0]
        size_y = size[1]
        pos = lvf.cMatrix[position]
        sizeVector= vector(size_x*10.5, size_y*10.5, 5)
        self.obstacle = box(size=sizeVector, pos = lvf.transformToVvector(pos) + sizeVector/2 - vector(5.25,5.25,0), color=color.orange, visible=obstacle_visible)
        lvf.setOccO_Call(size_x,size_y, position)
        lvf.remember(self.obstacle, obstacleDict)
        #print("Obstacle with size: " + str([size_x, size_y]) +" at position: " + str(position) + " created ")
        print("Objects.obstacle(" + str((size_x, size_y)) + "," + str(position) + "," + str(obstacle_visible) + ")")


        pass

    pass

class currentDebugBox:
    def __init__(self, coord):
        self.pos = lvf.cMatrix[coord]
        sizeVector = vector(1 * 10.5, 1 * 10.5, 5.1)
        self.obj = box(size=sizeVector, pos=lvf.transformToVvector(self.pos),
                       color=color.green, visible=True)
        lvf.remember(self.obj, showcaseDict)

class neighborDebugBox:
    def __init__(self, coord):
        self.pos = lvf.cMatrix[coord]
        sizeVector = vector(1 * 10.5, 1 * 10.5, 5)
        self.obj = box(size=sizeVector, pos=lvf.transformToVvector(self.pos),
                       color=color.blue, visible=True)
        lvf.remember(self.obj, showcaseDict)

class possiblePositionDebugBox:
    def __init__(self, coord):
        self.pos = lvf.cMatrix[coord]
        sizeVector = vector(1 * 10.5, 1 * 10.5, 5)
        self.obj = box(size=sizeVector, pos=lvf.transformToVvector(self.pos),
                       color=color.cyan, visible=True, opacity=0.5)
        lvf.remember(self.obj, showcaseDict)

class clamp:
    def __init__(self, coord,axis, clampVisible):
        self.pos = lvf.cMatrix[coord]
        sizeVector = vector(1,15,12)
        self.obj = cylinder(size=sizeVector ,axis = axis, pos=lvf.transformToVvector(self.pos),
                       color=color.black, visible=True)
        lvf.remember(self.obj, pipeDict)

# class displayText:
#     def __init__(self, pos):
#         self.pos = pos
#         self.T = label()
#     def changeText(self, string_sequence):
#         self.T.text = string_sequence
#     def display(self, string_sequence):
#         self.T = label(text=string_sequence, pos = self.pos, align="left", color=color.white, linewidth=5, background=color.black )
