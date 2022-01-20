from vpython import *
from rendering.object_rendering import render_corner

"""testing ground for whatever I need to test with vpython"""
pos = vector(0,0,0)
scene = canvas()
direction=(1,-1)
corner = render_corner(pos = pos, scene=scene, course=direction)

