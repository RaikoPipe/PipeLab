from dataclasses import dataclass, field
import vpython as vpy
from win32api import GetSystemMetrics

default_mounting_wall_dim = vpy.vector(1000,2000,150)

@dataclass
class scene_data:
    # camera
    camera_fov: float = vpy.pi/5
    camera_pos: vpy.vector = .5 * default_mounting_wall_dim

    # screen size (determined automatically by default)
    x_res : int = GetSystemMetrics(0)
    y_res : int = GetSystemMetrics(1)

    # lights
    ambient_light : vpy.vector = vpy.vector(1,1,1)
    distant_lights : tuple = ((vpy.vector(0.22, 0.44, 0.88), vpy.vector(1, 1, 1)),
                              (vpy.vector(-0.88, -0.22, -0.44), vpy.vector(1,1,1)))

    # background
    background_color : vpy.vector = vpy.color.black



@dataclass
class mounting_wall_data:
    """Data class that contains visual information of mounting_wall objects. Provides a default."""
    dim: vpy.vector = default_mounting_wall_dim
    #wrap_color: vpy.vector = vpy.vector(0.689, 0.515, 0.412)
    mount_color: vpy.vector = vpy.vector(0.8, 0.8, 0.8)
    mount_spot_color: vpy.vector = vpy.color.black
    mount_spot_radius: float =  10

@dataclass
class pipe_data:
    """Data class that contains computational and visual information of pipe object. Provides no default values."""
    # computational values
    point_length: int
    cost: float

    # visual values
    length: float
    color: vpy.vector
    radius: float = 25
    overhang_length: float = 45

# @dataclass
# class corner_data:
#     """Data class that contains computational and visual information of corner objects. Provides a default."""
#     # computational values
#     cost: float
#     point_length: int = 1
#     overhang_length : float = -80
#
#     # visual values
#     radius: float = 26
#     color: vpy.vector = vpy.color.gray

@dataclass
class obstacle_data:
    """Data class that contains visual information of obstacle objects. Provides a default."""
    dim : vpy.vector = vpy.vector(105,105,50)
    color: vpy.vector = vpy.color.orange










