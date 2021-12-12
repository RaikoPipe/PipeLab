import vpython as vpy
from rendering import object_data_classes as odc
import numpy as np
from typing import Optional
"""functions for rendering single objects (or compound objects)"""

testvector = vpy.vector(0,0,0)

#defaults
from win32api import GetSystemMetrics

default_mounting_wall = odc.mounting_wall_data()
default_scene = odc.scene_data()
default_obstacle = odc.obstacle_data()

# default pipe types
pipe_magenta = odc.pipe_data(point_length=1, cost = 1.15, length= 88, color= vpy.color.magenta)
pipe_blue = odc.pipe_data(point_length=2, cost = 1.38, length= 193, color= vpy.color.blue)
pipe_cyan = odc.pipe_data(point_length=3, cost = 1.6, length= 298, color= vpy.color.cyan)
pipe_green = odc.pipe_data(point_length=4, cost = 1.82, length= 403, color= vpy.color.green)
pipe_dark_green = odc.pipe_data(point_length=5, cost = 2.04, length= 508, color= vpy.vector(.5, 1, .5))
pipe_black = odc.pipe_data(point_length=6, cost = 2.50, length= 613, color=vpy.color.black)

default_pipes = {0: "corner not implemented", 1:pipe_magenta, 2: pipe_blue, 3:pipe_cyan, 4: pipe_green, 5:pipe_dark_green, 6: pipe_black}


def create_new_scene(scene_data: odc.scene_data = default_scene) -> vpy.canvas:
    """returns a new vpython scene with ambient light and basic distant light."""

    # create scene
    scene = vpy.canvas(background = scene_data.background_color, fov = scene_data.camera_fov, center = scene_data.camera_pos,
                           width = scene_data.x_res, height = scene_data.y_res)

    # set ambient light
    scene.ambient = scene_data.ambient_light

    # create some basic distant lighting
    scene.lights = [] #clear vpython default distant lights
    for distant_light in scene_data.distant_lights:
        scene.lights.append(vpy.distant_light(direction=distant_light[0], color = distant_light[1]))

    return scene


def render_mounting_wall(scene: vpy.canvas, rendering_grid: np.ndarray, mounting_wall_data : odc.mounting_wall_data = default_mounting_wall) -> (vpy.box,list):
    """Renders a basic mounting wall."""


    # initialise wall, set position, size, color
    mounting_wall_pos = 0.5*mounting_wall_data.dim
    scene.center = mounting_wall_pos

    mounting_wall = vpy.box(scene = scene, pos =mounting_wall_pos + vpy.vector(0, 0, 0.1),
                             size =mounting_wall_data.dim + vpy.vector(0, 0, 0.1), color = mounting_wall_data.mount_color)

    # render mounting spots on mounting wall
    dot_objects = []
    for mounting_spot in rendering_grid:
        for pos in mounting_spot:
            spot = vpy.cylinder(scene= scene, pos=pos, radius=mounting_wall_data.mount_spot_radius,
                                color = mounting_wall_data.mount_spot_color, axis=vpy.vector(0, 0, 5))
            dot_objects.append(spot)

    return mounting_wall, dot_objects

def render_obstacle(scene : vpy.canvas, pos: vpy.vector, obstacle_data: odc.obstacle_data = default_obstacle) -> vpy.box:
    """Renders an obstacle object."""

    obstacle = vpy.box(scene= scene, pos=pos, size=obstacle_data.dim, color=obstacle_data.color)

    return obstacle

# todo: handle corners
def render_pipe(scene : vpy.canvas, pos: vpy.vector, direction: tuple[int,int], pipe_data: Optional[odc.pipe_data], part_id: Optional[int] = None,
                )\
        -> vpy.cylinder:
    """Renders a pipe object."""
    if part_id is not None:
        pipe_data = default_pipes[part_id]

    # determine dimensions
    pos += vpy.vector(pipe_data.overhang_length*direction[0], pipe_data.overhang_length*direction[1], pipe_data.radius)
    size = vpy.vector(pipe_data.length, pipe_data.radius*2, pipe_data.radius*2)
    direction = vpy.vector(direction[0], direction[1], 0)

    pipe = vpy.cylinder(scene= scene, pos=pos, axis = direction, size=size, color=pipe_data.color)

    return pipe









