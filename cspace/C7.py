import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *

import C3

def pad_by_1_helper(cspace: np.array):
    """Pad the configuration space by one grid cell.

    Parameters
    ----------
    cspace : np.array
        The origianl configuration space of the robot.

    Returns
    -------
    np.array
        The padded configuration space of the robot.
    """
    padded_cspace = cspace.copy()
    # element in cspace is 1 if in collision, 0 if not

    assert padded_cspace.ndim == 2
    for cur_x in range(cspace.shape[0]):
        for cur_y in range(cspace.shape[1]):
            neighbor_candidates =  C3.filter_out_neighbors(cspace.shape, 
                                                           [[cur_x - 1, cur_y-1],
                                                            [cur_x - 1, cur_y],
                                                            [cur_x - 1, cur_y+1],

                                                            [cur_x, cur_y-1],
                                                            # [cur_x, cur_y],
                                                            [cur_x, cur_y+1],

                                                            [cur_x + 1, cur_y-1],
                                                            [cur_x + 1, cur_y],
                                                            [cur_x + 1, cur_y+1]])
            
            if math.isclose(cspace[cur_x, cur_y], 1):
                for neighbor_x, neighbor_y in neighbor_candidates:
                    padded_cspace[neighbor_x, neighbor_y] = 1

    return padded_cspace


def C7_func(cspace: np.array) -> np.array:
    """Pad the configuration space by one grid cell.

    Parameters
    ----------
    cspace : np.array
        The origianl configuration space of the robot.

    Returns
    -------
    np.array
        The padded configuration space of the robot.
    """

    ### Insert your code below: ###
    
    padded_cspace = pad_by_1_helper(cspace)
    return pad_by_1_helper(padded_cspace)
