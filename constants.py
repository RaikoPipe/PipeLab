from vpython import *

# directions
up = (0, 1)
down = (0, -1)
right = (1, 0)
left = (-1, 0)

valid_directions = {up, down, left, right}

horizontal_directions = {left, right}
vertical_directions = {up, down}

# vpy.vector directions
v_up = vector(0, 1, 0)
v_down = vector(0, -1, 0)
v_right = vector(1, 0, 0)
v_left = vector(-1, 0, 0)

to_vpy_vec = {up: v_up, down: v_down, right: v_right, left: v_left}

# rotations (counter clockwise)
rot_90 = pi / 2
rot_180 = pi
rot_270 = -pi / 2

fitting_id = 0
