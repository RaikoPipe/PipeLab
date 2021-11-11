import vpython as vpy
import object_data_classes as odc

"""functions for rendering single objects (or compound objects)"""

testvector = vpy.vector(0,0,0)

#defaults
from win32api import GetSystemMetrics

default_mounting_wall = odc.mounting_wall()
default_scene = odc.scene()
default_obstacle = odc.obstacle()

# default pipe types
pipe_magenta = odc.pipe(point_length=1, cost = 1.15, length= 88, color= vpy.color.magenta)
pipe_blue = odc.pipe(point_length=2, cost = 1.38, length= 193, color= vpy.color.blue)
pipe_cyan = odc.pipe(point_length=3, cost = 1.6, length= 298, color= vpy.color.cyan)
pipe_green = odc.pipe(point_length=4, cost = 1.82, length= 403, color= vpy.color.green)
pipe_dark_green = odc.pipe(point_length=5, cost = 2.04, length= 508, color= vpy.vector(.5,1,.5))


def create_new_Scene(scene_data: odc.scene = default_scene) -> vpy.scene:
    """returns a new vpython scene with ambient light and basic distant light."""

    # create scene
    scene = vpy.scene(background = scene_data.background_color, fov = scene_data.camera_fov, center = scene_data.camera_pos,
                           width = scene_data.x_res, height = scene_data.y_res)

    # set ambient light
    scene.ambient = scene_data.ambient_light

    # create some basic distant lighting
    scene.lights = [] #clear vpython default distant lights
    for distant_light in scene_data.distant_lights:
        scene.lights.append(vpy.distant_light(direction=distant_light[0], color = distant_light[1]))

    return scene_data


def render_mounting_wall(scene: vpy.scene, coord_grid, mounting_wall_data : odc.mounting_wall = default_mounting_wall) -> vpy.compound:
    """Renders a basic mounting wall."""

    compound_objects = []

    # initialise wall, set position, size, color
    wall_wrap = vpy.box(scene = scene, pos=mounting_wall_data.dim * .5, size=mounting_wall_data.dim,
                        color=mounting_wall_data.wrap_color, shininess=0.0)

    mounting_wall = vpy.box(scene = scene, pos =0.5 * mounting_wall_data.dim + vpy.vector(0, 0, 0.1),
                             size =mounting_wall_data.dim + vpy.vector(0, 0, 0.1), color = mounting_wall_data.mount_color)

    compound_objects.append(wall_wrap)
    compound_objects.append(mounting_wall)

    # render mounting spots on mounting wall
    for mounting_spot in coord_grid:
        spot = vpy.cylinder(scene= scene, pos=mounting_spot, radius=mounting_wall_data.mount_spot_radius,
                            color = mounting_wall_data.mount_spot_color, axis=(0, 0, 5))
        compound_objects.append(spot)

    wall_compound = vpy.compound(compound_objects)

    return wall_compound

def render_obstacle(scene : vpy.scene, pos: vpy.vector, obstacle_data: odc.obstacle = default_obstacle) -> vpy.box:
    """Renders an obstacle object."""

    obstacle = vpy.box(scene= scene, pos=pos, size=obstacle_data.dim, color=obstacle_data.color)

    return obstacle

def render_pipe(pos: vpy.vector, axis:vpy.vector, scene : vpy.scene, pipe_data: odc.pipe) -> vpy.cylinder:
    """Renders a pipe object."""

    #todo: create function get_directional_overhang
    directional_overhang = get_directional_overhang(pipe_data.overhang_length, axis)

    pipe = vpy.cylinder(pos = pos + directional_overhang, size = vpy.vector(pipe_data.length, 2*pipe_data.radius, 2*pipe_data.radius),
                        color = pipe_data.color)

    return pipe

    






