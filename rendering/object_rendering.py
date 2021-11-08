import vpython as vpy

"""functions for rendering specific vpython objects"""

testvector = vpy.vector(0,0,0)

#defaults
from win32api import GetSystemMetrics

default_light = {"ambient_light": vpy.vector(1,1,1),
                 "distant_lights": [(vpy.vector(0.22,  0.44,  0.88), vpy.vector(1,1,1)),
                                    (vpy.vector(-0.88, -0.22, -0.44), vpy.vector(1,1,1))]}
#fixme: radius of mounting spot
default_mounting_wall = {"dim": vpy.vector(1000,2000,150), "wrap_color": vpy.vector(0.689, 0.515, 0.412),
                         "mount_color": vpy.vector(0.8, 0.8, 0.8), "mount_spot_color": vpy.color.black,
                         "mount_spot_radius": 80}

default_camera = {"fov": vpy.pi/5, "pos": 0.5 * default_mounting_wall["dim"]}

default_screen = {"x_res": GetSystemMetrics(0), "y_res" : GetSystemMetrics(1)}

default_obstacle = {"dim": vpy.vector(10,10,2), "color": vpy.color.orange}

def create_new_Scene(lighting: dict = default_light, background_color = vpy.color.white, camera_dim = default_camera,
                     screen_dim = default_screen) -> vpy.scene:
    """returns a new vpython scene with ambient light and basic distant light"""

    # create scene
    scene = vpy.scene(background = background_color, fov = camera_dim["fov"], center = camera_dim["pos"],
                      width = screen_dim["x_res"], height = screen_dim["y_res"])

    # set ambient light
    scene.ambient = lighting["ambient_light"]

    # create some basic distant lighting
    scene.lights = [] #clear vpython default distant lights
    for distant_light in lighting["distant_lights"]:
        scene.lights.append(vpy.distant_light(direction=distant_light[0], color = distant_light[1]))

    return scene


def render_mounting_wall(coord_grid, scene = vpy.scene, mounting_wall_params: dict= default_mounting_wall, ) -> vpy.compound:
    """Render a base wall according to given parameters on a Vpython-scene"""

    wall_dim = mounting_wall_params["dim"]
    wall_pos = 0.5 * mounting_wall_params["dim"]
    mount_color = mounting_wall_params["mount_color"]
    wrap_color = mounting_wall_params["wrap_color"]


    compound_objects = []

    # initialise wall, set position, size, color
    wall_wrap = vpy.box(scene = scene, pos=wall_pos, size=wall_dim,
                       color=wrap_color, shininess=0.0)

    mounting_wall = vpy.box(scene = scene, pos = 0.5 * wall_dim + vpy.vector(0,0,0.1),
                                  size =  wall_dim + vpy.vector(0,0,0.1), color = mount_color)

    compound_objects.append(wall_wrap, mounting_wall)

    # render mounting spots on mounting wall
    spot_color = mounting_wall_params["mount_spot_color"]
    spot_radius = mounting_wall_params["mount_spot_radius"]
    for mounting_spot in coord_grid:
        spot = vpy.cylinder(scene= scene, pos=mounting_spot, radius=spot_radius, color = spot_color, axis=(0,0,5))
        compound_objects.append(spot)

    wall_compound = vpy.compound(compound_objects)

    return wall_compound

def render_obstacle(pos: vpy.vector, scene = vpy.scene, obstacle_parameters = default_obstacle) -> vpy.box:
    """renders an obstacle object"""
    dim = obstacle_parameters["dim"]
    color = obstacle_parameters["color"]

    obstacle = vpy.box(scene= scene, pos=pos, size=dim, color=color)

    return obstacle
    






