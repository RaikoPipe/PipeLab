from vpython import *
import LogicalVfunctions as lvf

scene = canvas(background= color.white)
cornerPipeTemp1 = cylinder(size=vector(11.2, 6, 6), color=color.black, axis=lvf.right, shininess = 0)
cornerPipeTemp2 = cylinder(size=vector(11.2, 6, 6), color=color.black, axis=lvf.up, pos=vector(11.2 - 3, 0, 0), shininess = 0)
cornerTemp = compound([cornerPipeTemp1, cornerPipeTemp2])
cornerTemp.axis = vector(0,0,-1)