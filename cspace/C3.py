import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
import typing
import copy
import math

from collections import deque

def get_closest_index(lst, number):
    assert(lst.ndim == 1)

    closest_index = 0
    for i in range(lst.shape[0]):
        if abs(lst[i] - number) < abs(lst[closest_index] - number):
            closest_index = i
    
    return closest_index


def get_closest_goal_coords(q_grid, q_goal):
    return [get_closest_index(q_grid, q_goal[0]), get_closest_index(q_grid, q_goal[1])]




def filter_out_neighbors(shape, neighbors_lst):
    return [[x, y] for x, y in neighbors_lst if (0 <= x and x < shape[0] 
                                                 and 0 <= y and y < shape[1])]


def waveform_planner(cspace: np.array, q_grid: np.array, q_goal):
    distances = copy.deepcopy(cspace)
    assert(distances.ndim == 2)
    x, y = distances.shape


    # set obstacle points to val of 1
    for i in range(x):
        for j in range(y):
            # element in cspace is 1 if in collision, 0 if not
            if (math.isclose(cspace[i,j], 1, rel_tol=1e-5)):
                # print([q_grid[i], q_grid[j]])
                distances[i,j] = 1
    

    """
    1. L={goal state}, d(goal state) = 2, d(obstacle
    states) = 1, d(rest of states) = 0
    2. while L != null
    3. pop item i from L
    4. for all neighbors j of i such that d(j)==0
    5. d(j) = d(i)+1
    6. push j onto L
    """

    goal_coordinates = get_closest_goal_coords(q_grid, q_goal)
    print(goal_coordinates)
    todo_lst = deque()
    todo_lst.append(goal_coordinates)
    distances[goal_coordinates[0], goal_coordinates[1]] = 2

    while todo_lst:
        cur_coords = todo_lst.popleft()
        # print(cur_coords)
        cur_x, cur_y = cur_coords

        # print(distances[cur_x, cur_y])

        neighbor_coords = filter_out_neighbors(distances.shape, [#[cur_x - 1, cur_y-1],
                                                                [cur_x - 1, cur_y],
                                                                #[cur_x - 1, cur_y+1],

                                                                [cur_x, cur_y-1],
                                                                # [cur_x, cur_y],
                                                                [cur_x, cur_y+1],

                                                                #[cur_x + 1, cur_y-1],
                                                                [cur_x + 1, cur_y],
                                                                #[cur_x + 1, cur_y+1]
                                                                ])
        
        for neigh_x, neigh_y in neighbor_coords:
            if distances[neigh_x, neigh_y] == 0:
                distances[neigh_x, neigh_y] = distances[cur_x, cur_y] + math.dist([q_grid[cur_x], q_grid[cur_y]], 
                                                                                  [q_grid[neigh_x], q_grid[neigh_y]])
                todo_lst.append([neigh_x, neigh_y])

    x, y = distances.shape
    # cleanup and set rest of untouched distances to 0
    for i in range(x):
        for j in range(y):
            # element in cspace is 1 if in collision, 0 if not
            if (math.isclose(distances[i,j], 0, rel_tol=1e-5)):
                # print([q_grid[i], q_grid[j]])
                distances[i,j] = 1

    return distances



def C3_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array,q_grid: np.array, q_goal: np.array) -> np.array:
    """Create a new 2D array that shows the distance from each point in the configuration space to the goal configuration.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        The configuration space of the robot given by C2. The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.
    q_goal : np.array
        A 2 x 1 numpy array representing the goal configuration of the robot in the format of [q1, q2].

    Returns
    -------
    np.array
       A 2D numpy array representing the distance from each cell in the configuration space to the goal configuration. 
       The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    """

    ### Insert your code below: ###
    # shape1, shape2, pivot1, pivot2 = q2poly(robot, q_goal)
    # ex q_goal = [3.05, 0.05]

    return waveform_planner(cspace, q_grid, q_goal)

    # distances = copy.deepcopy(cspace)
    # assert(distances.ndim == 2)
    # x, y = distances.shape

    # for i in range(x):
    #     for j in range(y):
    #         # print(cspace[i,j])
    #         # print(f"{i} {j}: {cspace[i,j]}")
    #         # element in cspace is 1 if in collision, 0 if not
    #         if (math.isclose(cspace[i,j], 0, rel_tol=1e-5)):
    #             # print([q_grid[i], q_grid[j]])
    #             distances[i,j] = math.dist([q_grid[i], q_grid[j]], q_goal)
    #         else:
    #             # in collision, equals 1
    #             distances[i,j] = 0 # why?????????

    # return distances
