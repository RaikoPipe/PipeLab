from dataclasses import dataclass, field
from typing import Any
import vpython as vpy
from win32api import GetSystemMetrics

default_mounting_wall_dim = vpy.vector(1000, 2000, 150)


def default_camera_pos() -> Any:
    return 0.5 * default_mounting_wall_dim


def default_distant_lights() -> tuple:
    return ((vpy.vector(0.22, 0.44, 0.88), vpy.vector(1, 1, 1)),
            (vpy.vector(-0.88, -0.22, -0.44), vpy.vector(1, 1, 1)))


@dataclass
class SceneData:
    # camera
    camera_fov: float = vpy.pi / 5
    camera_pos: vpy.vector = field(default_factory=default_camera_pos)

    # screen size (determined automatically by default)
    x_res: int = field(default_factory=lambda: GetSystemMetrics(0))
    y_res: int = field(default_factory=lambda: GetSystemMetrics(1))

    # lights
    ambient_light: vpy.vector = field(default_factory=lambda: vpy.vector(1, 1, 1))
    distant_lights: tuple = field(default_factory=default_distant_lights)

    # background
    background_color: vpy.vector = field(default_factory=lambda: vpy.color.black)


@dataclass
class MountingWallData:
    """Data class that contains visual information of mounting_wall objects. Provides a default."""
    dim: vpy.vector = field(default_factory=lambda: default_mounting_wall_dim)
    mount_color: vpy.vector = field(default_factory=lambda: vpy.vector(0.8, 0.8, 0.8))
    mount_spot_color: vpy.vector = field(default_factory=lambda: vpy.color.black)
    mount_spot_radius: float = 10


@dataclass
class PipeData:
    """Data class that contains computational and visual information of pipe object. Provides no default values."""
    # computational values
    point_length: int
    cost: float

    # visual values
    length: float
    color: vpy.vector
    radius: float = 25
    overhang_length: float = 45


@dataclass
class ObstacleData:
    """Data class that contains visual information of obstacle objects. Provides a default."""
    dim: vpy.vector = field(default_factory=lambda: vpy.vector(105, 105, 50))
    color: vpy.vector = field(default_factory=lambda: vpy.color.orange)