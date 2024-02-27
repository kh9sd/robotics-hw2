import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
import typing

import C3

def C4_func(distances: np.array,q_grid: np.array, q_start: np.array) -> typing.List[np.array]:
    """Using the distance array from C3, find the optimal path from the start configuration to the goal configuration (zero value).

    Parameters
    ----------
    distances : np.array
        A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration.
        This is given by C3 
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_start : np.array
        A 2 x 1 numpy array representing the start configuration of the robot in the format of [q1, q2].

    Returns
    -------
    typing.List[np.array]
        A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using indices of q_grid.
        Example: [ [q1_0 , q2_0], [q1_1, q2_1], .... ]
    """
    # 1 is obstacle
    # 2 is goal

    ### Insert your code below: ###
    cur_x, cur_y = C3.get_closest_goal_coords(q_grid, q_start)
    result_path = [np.array([cur_x, cur_y])]

    while not math.isclose(distances[cur_x, cur_y], 2):
        neighbor_candidates =  [[cur_x - 1, cur_y-1],
                                [cur_x - 1, cur_y],
                                [cur_x - 1, cur_y+1],

                                [cur_x, cur_y-1],
                                # [cur_x, cur_y],
                                [cur_x, cur_y+1],

                                [cur_x + 1, cur_y-1],
                                [cur_x + 1, cur_y],
                                [cur_x + 1, cur_y+1]]
        
        closest_neighbor = sorted([neighbor for neighbor in neighbor_candidates 
                            if not math.isclose(distances[neighbor[0],
                                                        neighbor[1]],
                                                        1)], # this means its an obstacle
                            key=lambda neighbor: distances[neighbor[0], neighbor[1]])[0]
        cur_x, cur_y = closest_neighbor
        result_path.append(np.array([cur_x, cur_y]))

    return result_path
