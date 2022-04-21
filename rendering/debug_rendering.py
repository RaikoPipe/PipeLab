import numpy as np
import vpython as vpy


def display_pos(scene: vpy.canvas, rendering_grid: np.ndarray):
    texts = []
    for pos, coord in np.ndenumerate(rendering_grid):
        text = vpy.label(text=str(pos), pos=coord, color=vpy.color.white, scene=scene)
        texts.append(text)
    return texts
