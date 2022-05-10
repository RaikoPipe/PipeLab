from vpython import vector, pi

#: algorithms
mca_star = "mca*"
mcsa_star = "mcsa*"
dijkstra = "dijkstra"
best_first_search = "best_first"


#: directions
up = (0, 1)  #: +y
down = (0, -1)  #: -y
right = (1, 0)  #: +x
left = (-1, 0)  #: -x

valid_directions = {up, down, left, right}  #: all used directions in PipeLab modules

horizontal_directions = {left, right}  #: all horizontal directions used in PipeLab modules
vertical_directions = {up, down}  #: all vertical directions used in PipeLab modules

#: vpy.vector directions
v_up = vector(0, 1, 0) #: Vpython up
v_down = vector(0, -1, 0) #: Vpython down
v_right = vector(1, 0, 0) #: Vpython right
v_left = vector(-1, 0, 0) #: Vpython left

to_vpy_vec = {up: v_up, down: v_down, right: v_right, left: v_left} #: dict for transforming tuple directions to Vpython directions

#: rotations (counter clockwise)
rot_90 = pi / 2 #: rotate by 90°
rot_180 = pi #: rotate by 180°
rot_270 = -pi / 2 #: rotate by 270°

fitting_id = 0 #: fitting id convention

#: event codes
fit_event_code = 1
pipe_event_code = 2
att_event_code = 3
pick_event_code = 4
pick_manual_event_code = 5

