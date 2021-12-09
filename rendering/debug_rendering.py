import object_rendering
import group_rendering
from grid import grid_functions, randomizer

x=20
y=20

r_grid, mounting_wall_data = grid_functions.get_rendering_grid(x,y)
state_grid = grid_functions.get_empty_stategrid(x,y)
state_grid = randomizer.set_random_obstacles(0.1, state_grid)

canvas = object_rendering.create_new_scene()
mounting_wall, dot_object = \
    object_rendering.render_mounting_wall(scene=canvas,rendering_grid=r_grid, mounting_wall_data=mounting_wall_data)

obstacles = group_rendering.render_obstacles_from_state_grid(state_grid=state_grid, rendering_grid=r_grid, scene=canvas)