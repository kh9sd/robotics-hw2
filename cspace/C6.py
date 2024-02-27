import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Polygon
from helper_functions import *
from q2poly import q2poly
import shapely
from shapely.geometry import Polygon as Polygon_shapely
from shapely import MultiPoint
import typing

from C1 import C1_func




def C6_func(robot: typing.Dict[str, typing.List[float]], q_path: typing.List[np.array], obstacles: typing.List[Polygon]) -> int:
    """Calculate the number of collisions that occur along the path.
    
    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters.
    q_path : typing.List[np.array]
       A list of 2 x 1 numpy array representing the path from the start configuration to the goal configuration using actual angle values.
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles.

    Returns
    -------
    int
        The number of collisions that occur along the path.
    """

    ### Insert your code below: ###
    num_collisions = 0

    obstacle_shape = shapely.union_all(list(map(Polygon_shapely, obstacles)))

    path_iter = iter(q_path)

    try:
        from_config = next(path_iter)
        while(True):
            to_config = next(path_iter)

            from_shape1, from_shape2, _, _ = q2poly(robot, from_config)
            to_shape1, to_shape2, _, _ = q2poly(robot, to_config)

            hull = shapely.convex_hull(MultiPoint([*from_shape1, *from_shape2, *to_shape1, *to_shape2]))

            if (hull.intersects(obstacle_shape)):
                num_collisions += 1

            from_config = to_config
    except StopIteration:
        pass

    return num_collisions
