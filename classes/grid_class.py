from path_finding import state_grid_functions as sgf
from rendering import coord_grid_functions as cgf

class GridPreset:
    """class that contains the state_grid, coord_grid and solutions to it"""
    def __init__(self):
        self.state_grid = None
        self.coord_grid = None
        self.solutions = []

    def init_empty_grid(self, x_length, y_length):
        self.state_grid = sgf.get_empty_stategrid(x_length, y_length)
        self.coord_grid = cgf.get_coord_grid(x_length, y_length)