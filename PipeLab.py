from vpython import vector, color
import numpy as np
import rendering
from path_finding import path_finder
from grid import grid_functions

# todo: finish later

class PipeLab:
    solution_list: dict  # {solution: score}
    """An instance of PipeLab acts as an interface for path finding, rendering and user IO"""
    def __init__(self):

        self.coord_grid = None
        self.state_grid = None

        self.solution_list = {}

        # self.wallDim = wallShape
        # self.topDim = topShape
        # self.wallColor = vector(0.8, 0.8, 0.8)
        # self.topColor = vector(0.8, 0.8, 0.8)
        # self.dotColor = color.black
        # self.WallVisible = wallVisible
        # self.topVisible = topVisible
        # self.obstacleVisible = obstacleVisible
        # self.pipeVisible = pipeVisible
        # self.lampVisible = lampvisible
        # self.topDotVisible = topDotVisible
        # self.wallDotVisible = wallDotVisible
        # self.coordinateInfoVisible = coordinateInfoVisible
        # self.camera = camera
        # self.SceneColor = backgroundColor
        # self.xDots, self.yDots,self.shiftPos = calcDots(self.wallDim.x,self.wallDim.y, self.wallDim.z, self.topDim.y)
        # self.lMatrix = np.tile(0, (self.xDots, self.yDots))
        # self.posMatrix = lvf.createPmatrix(self.xDots, self.yDots,dot_distFromwall_x,dot_distFromWallBottom_y, dot_distance)
        # self.start = None
        # self.goal = None
        # self.availableTypes = availabeTypes
        # self.altMatrix = None

    def find_solution(self, solution_preferences:dict, available_parts:dict, state_grid)->dict:
        """finds a solution to the given conditions"""
        if solution_preferences["algorithm"] == "dijkstra":
            path, parts_used, solution_grid = path_finder.dijkstra(self.lMatrix, self.start, self.goal, self.shiftPos, self.startAxis, self.goalAxis, False,
                                                                   False, "", 1, self.yDots, self.availableTypes, False)
        elif solution_preferences["algorithm"] == "best-first":
            path, parts_used, solution_grid = path_finder.bestFirstSearch(self.lMatrix, self.start, self.goal, self.shiftPos, self.startAxis, self.goalAxis, False,
                                                                          False, "", 1, self.yDots, self.availableTypes, False)
        elif solution_preferences["algorithm"] == "multicriteria astar":
            path, parts_used, solution_grid = path_finder.multicriteriaAstar(self.lMatrix, self.start, self.goal, self.shiftPos, self.startAxis, self.goalAxis, False,
                                                                             False, "", 1, self.yDots, self.availableTypes, False, gC, gP, gMinO)
        else:
            path, parts_used, solution_grid = path_finder.astar(self.lMatrix, self.start, self.goal, self.shiftPos, self.startAxis, self.goalAxis, False,
                                                                False, "", 1, self.yDots, self.availableTypes, False)
        solution_package = {"path":path,"parts_used":parts_used, "solution_grid":solution_grid}

        self.solution_list.append(solution_package)

        return solution_package



