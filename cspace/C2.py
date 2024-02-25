import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from shapely import intersects, union
from shapely.geometry import Polygon as Polygon_shapely
from helper_functions import *
from q2poly import q2poly
import typing

def polygon_intersects_any_in_lst(poly, poly_list):
    return any(intersects(poly, other_poly) for other_poly in poly_list)

def bool_to_num(b):
    if b == True:
        return 1
    elif b == False:
        return 0
    
    assert False

def C2_func(robot: typing.Dict[str, typing.List[float]], cspace: np.array, obstacles: typing.List[Polygon],q_grid: np.array) -> np.array:
    """Create the configuration space for the robot with the given obstacles in the given empty cspace array.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    cspace : np.array
        An empty 2D numpy array
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles
    q_grid : np.array
        A R x 1 numpy array representing the grid over the angle-range. R is the resolution.

    Returns
    -------
    np.array
        A 2D numpy array representing the updated configuration space. The first dimension is q1 and the second dimension is q2. Example: [q1, q2]
    """

    ### Insert your code below: ###

    lenghasd = q_grid.shape[0]
    result = np.zeros((lenghasd, lenghasd))

    # its a fucking lie its not a fucking list of Polygons they pass in
    obstacles = list(map(Polygon_shapely, obstacles))

    for i, angle1 in enumerate(q_grid):
        for j, angle2 in enumerate(q_grid):
            link_shape_1, link_shape_2, _, _ = q2poly(robot, [angle1, angle2])
            robot_poly = union(Polygon_shapely(link_shape_1), Polygon_shapely(link_shape_2))

            result[i,j] = polygon_intersects_any_in_lst(robot_poly, obstacles)

    return result
