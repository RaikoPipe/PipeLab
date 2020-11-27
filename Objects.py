from vpython import *
import numpy as np
from win32api import GetSystemMetrics
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

# this function creates the canvas, wall and top and dots

def PipeLabInstance(wall_shape, top_shape, wall_thickness, wall_color, top_color, lamp_visible,
             wall_visible, top_visible, coordinate_info_visible, camera_pos, background_color,x_res,y_res):
    main.scene.delete()

    # creating the scene

    main.scene = canvas(width=x_res, height=y_res, center=camera_pos, background=background_color, fov=pi/5)
    print("Detected resolution: " + str(x_res) + "x" + str(y_res))


    # calculating some vectors needed for Projection, in case wall changes size
    z_vector = vector(0, 0, 1)  # needed for projection
    x_vector = vector(1, 0, 0)  # needed for projection

    # initialise top, set position for top_shape relative to its own size and wall x and z coordinate,
    wallHeight = comp(wall_shape, vector(0,1,0))
    wallWidth = comp(wall_shape, vector(1,0,0))
    topHeight=comp(top_shape, vector(0,1,0))
    top_pos = vector(0.5*wallWidth,wallHeight+0.5*topHeight,0.5*wall_thickness)
    # top_pos = 0.5 * wall_shape + 0.75 * proj(wall_shape, vector(0, 1,
    #                                                             0))  # + vector(-0,150,0)#proj(wall_shape, vector(1,0,0)) + vector(50,0,0)

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

    obstacleList = [] #will be used to identify obstacles later



def closeScene():
    try:
        scene.delete()
    except: Exception



def StartEndInt(start_position, end_position, start_direction, end_direction, background_color, wallvisible, topvisible):
    if background_color == color.black:
        text_color = color.white
    else: text_color = color.black
    # initialise Start and End position Cylinders
    startPos = lvf.cMatrix[start_position]
    endPos = lvf.cMatrix[end_position]


    startcylinder = cylinder(pos=lvf.transformToVvector(startPos) + vector(0, 0, 5) - start_direction,axis=start_direction, size=vector(1, 5, 5),
                             color=color.green, visible = wallvisible)
    start_label = label(pos=lvf.transformToVvector(startPos), xoffset = 30, text="Start", space=30,
                        height=16, border=4,
                        font="sans", color=text_color, visible = wallvisible)
    lvf.remember(startcylinder, wallDict)
    lvf.remember(start_label, wallDict)

    endcylinder = cylinder(pos=lvf.transformToVvector(endPos) + vector(0, 0, 5) - end_direction, axis=end_direction, size=vector(1, 5, 5),
                           color=color.orange, visible = topvisible)
    end_label = label(pos=lvf.transformToVvector(endPos), xoffset = 30, text="Goal", space=30,
                      height=16, border=4,
                      font="sans", color=text_color, visible= topvisible)
    lvf.remember(endcylinder, topDict)
    lvf.remember(end_label, topDict)

    pass




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

# this class creates all objects except wall, top and obstacles and checks if they can be placed
class pipe:
    def __init__(self, type, pipe_coord, pipe_axis, pipe_visible):
        self.pipe_pos = lvf.cMatrix[pipe_coord]
        self.pipe_ax = pipe_axis
        if type == "red+red":
            self.pipe_length = 33.5
            self.pipe_width = 5
            self.overhang = 4.5
            self.overhang2 = -4.5
            self.dotlength= 7
            self.secondDotDistance = 6
            self.socketDotDistance = 3
            self.pipe_pos2 = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, self.secondDotDistance)]
            self.directionalOverhang1 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
            self.directionalOverhang2 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang2,
                                                                         self.pipe_width)
            # if validPlacement(position, pipe_length) == True
            redpipe1 = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang1, axis=pipe_axis,
                     size=vector(self.pipe_length, self.pipe_width, self.pipe_width),
                     color=color.red, visible=pipe_visible)
            redpipe2= cylinder(pos=lvf.transformToVvector(self.pipe_pos2) + self.directionalOverhang2, axis=-pipe_axis,
                     size=vector(self.pipe_length, self.pipe_width, self.pipe_width),
                     color=color.red, visible=pipe_visible)
            createSocket(pipe_coord, pipe_axis, self.socketDotDistance, pipe_visible)
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
            print("red+red created at position: " + str(pipe_coord))
            lvf.remember(redpipe1, pipeDict)
            lvf.remember(redpipe2, pipeDict)

        elif type == "red+yellow":
            self.pipe_length1 = 33.5
            self.pipe_length2 = 23
            self.pipe_width = 5
            self.overhang = 4.5
            self.overhang2= -4.5
            self.dotlength= 6
            self.secondDotDistance = 5
            self.socketDotDistance = 3
            self.pipe_pos2 = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, self.secondDotDistance)]
            self.directionalOverhang1 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
            self.directionalOverhang2 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang2,
                                                                         self.pipe_width)
            # if validPlacement(position, pipe_length) == True
            redpipe1 = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang1, axis=pipe_axis,
                     size=vector(self.pipe_length1, self.pipe_width, self.pipe_width),
                     color=color.red, visible=pipe_visible)
            yellowpipe2= cylinder(pos=lvf.transformToVvector(self.pipe_pos2) + self.directionalOverhang2, axis=-pipe_axis,
                     size=vector(self.pipe_length2, self.pipe_width, self.pipe_width),
                     color=color.yellow, visible=pipe_visible)
            createSocket(pipe_coord, pipe_axis, self.socketDotDistance, pipe_visible)
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
            print("red+yellow created at position: " + str(pipe_coord))
            lvf.remember(redpipe1, pipeDict)
            lvf.remember(yellowpipe2, pipeDict)
        elif type == "yellow+yellow":
            self.pipe_length1 = 23
            self.pipe_length2 = 23
            self.pipe_width = 5
            self.overhang = 4.5
            self.overhang2 = -4.5
            self.dotlength= 5
            self.secondDotDistance = 4
            self.socketDotDistance = 2
            self.pipe_pos2 = lvf.cMatrix[lvf.determineSecondPipePlacement(pipe_axis, pipe_coord, self.secondDotDistance)]
            self.directionalOverhang1 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
            self.directionalOverhang2 = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang2, self.pipe_width)

            yellowpipe1 = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang1, axis=pipe_axis,
                     size=vector(self.pipe_length1, self.pipe_width, self.pipe_width),
                     color=color.yellow, visible=pipe_visible)
            yellowpipe2= cylinder(pos=lvf.transformToVvector(self.pipe_pos2) + self.directionalOverhang2, axis=-pipe_axis,
                     size=vector(self.pipe_length2, self.pipe_width, self.pipe_width),
                     color=color.yellow, visible=pipe_visible)
            createSocket(pipe_coord, pipe_axis, self.socketDotDistance, pipe_visible)
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
            print("yellow+yellow created at position: " + str(pipe_coord))
            lvf.remember(yellowpipe1, pipeDict)
            lvf.remember(yellowpipe2, pipeDict)


        elif type == "blue":
            self.pipe_length = 29.8
            self.pipe_width = 5
            self.overhang = 4.5
            self.dotlength = 3
            self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
            # if validPlacement(position, pipe_length) == True
            blue = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang, axis=pipe_axis,
                     size=vector(self.pipe_length, self.pipe_width, self.pipe_width),
                     color=color.blue, visible=pipe_visible)
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
            print("blue created at position: " + str(pipe_coord))
            lvf.remember(blue, pipeDict)

        elif type == "green":
            self.pipe_length = 19.3
            self.pipe_width = 5
            self.overhang = 4.5
            self.dotlength = 3
            self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
            # if validPlacement(position, pipe_length) == True
            green = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang, axis=pipe_axis,
                     size=vector(self.pipe_length, self.pipe_width, self.pipe_width),
                     color=color.green, visible=pipe_visible)
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
            print("green created at position: " + str(pipe_coord))
            lvf.remember(green, pipeDict)

        elif type == "purple":
            self.pipe_length = 8.8
            self.pipe_width = 5
            self.overhang = 4.5
            self.dotlength = 1
            self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_width)
            # if validPlacement(position, pipe_length) == True
            purple = cylinder(pos=lvf.transformToVvector(self.pipe_pos) + self.directionalOverhang, axis=pipe_axis,
                     size=vector(self.pipe_length, self.pipe_width, self.pipe_width),
                     color=color.purple, visible=pipe_visible)
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)

            print("purple created at position: " + str(pipe_coord))
            lvf.remember(purple, pipeDict)

        elif type == "corner":
            self.pipelength = ""
            self.pipe_radius= 2.6
            self.overhang = -8
            self.dotlength = 1
            self.directionalOverhang = lvf.determineDirectionalOverhang(type, pipe_axis, self.overhang, self.pipe_radius)
            'create corner pipe at desired position, if there is enough space'
            arc = shapes.arc(radius=self.pipe_radius, angle1=0, angle2=2 * pi, rotate = pi)
            CornerPipe = extrusion(path=paths.arc(radius=self.pipe_radius*3, angle1=-0.09, angle2=pi / 2 + 0.092), shape=arc,
                                   color=color.white, visible =pipe_visible)
            self.corner = compound([CornerPipe], up=vec(0, 0, 1), axis=pipe_axis, pos= lvf.transformToVvector(self.pipe_pos)+ self.directionalOverhang, visible=pipe_visible)
            print("corner created at position: " + str(pipe_coord))
            lvf.setOccP_Call(pipe_coord, self.dotlength, self.pipe_ax)
            lvf.remember(self.corner, pipeDict)
        else:
            sizeVector= vector(1*10.5, 1*10.5, 5)
            box(size=sizeVector, pos = lvf.transformToVvector(self.pipe_pos), color=color.red, visible=True, opacity = 0.9)
            #lvf.setOccO_Call(1,1, self.pipe_pos)
            print("Error" +" at position: " + str(pipe_coord))


    pass


# create obstacles
class obstacle:
    def __init__(self, size, position, obstacle_visible):
        size_x = size[0]
        size_y = size[1]
        pos = lvf.cMatrix[position]
        sizeVector= vector(size_x*10.5, size_y*10.5, 5)
        self.obstacle = box(size=sizeVector, pos = lvf.transformToVvector(pos) + sizeVector/2 - vector(5.25,5.25,0), color=color.orange, visible=obstacle_visible)
        lvf.setOccO_Call(size_x,size_y, position)
        print("Obstacle with size: " + str([size_x, size_y]) +" at position: " + str(position) + " created ")
        lvf.remember(self.obstacle, obstacleDict)

        pass

    pass



# class cursor:
#     def __init__(self, start_direction, start_position):
#         self.currentDir = start_direction
#         self.cursorarrow = arrow(pos=lvf.transformToVvector(start_position) + vector(0, 0, 5) + start_direction, color=color.yellow,
#                                  axis=start_direction, headlength = 5, headwidth = 5)
#
#     def Change(self, new_position, new_direction):
#         self.cursorarrow.pos = lvf.transformToVvector(new_position) + vector(0,0,5)
#         self.cursorarrow.axis = new_direction

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