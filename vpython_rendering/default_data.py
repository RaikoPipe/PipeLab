import vpython as vpy

from vpython_rendering import object_data_classes as odc

# defaults
default_mounting_wall = odc.MountingWallData()
default_scene = odc.SceneData()
default_obstacle = odc.ObstacleData()

# default pipe type_dictionary
pipe_magenta = odc.PipeData(point_length=1, cost=1.15, length=88, color=vpy.color.magenta)
pipe_blue = odc.PipeData(point_length=2, cost=1.38, length=193, color=vpy.color.blue)
pipe_cyan = odc.PipeData(point_length=3, cost=1.6, length=298, color=vpy.color.cyan)
pipe_green = odc.PipeData(point_length=4, cost=1.82, length=403, color=vpy.color.green)
pipe_dark_green = odc.PipeData(point_length=5, cost=2.04, length=508, color=vpy.vector(.5, 1, .5))
pipe_black = odc.PipeData(point_length=6, cost=2.50, length=613, color=vpy.color.black)
pipe_corner = odc.PipeData(point_length=1, cost=1.15, length=88, color=vpy.color.gray(0.2), radius=24)
default_pipes = {1: pipe_magenta, 2: pipe_blue, 3: pipe_cyan, 4: pipe_green, 5: pipe_dark_green, 6: pipe_black}
