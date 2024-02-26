import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from q2poly import q2poly
from helper_functions import *
import typing


def C1_func(robot: typing.Dict[str, typing.List[float]], q: typing.List[float], obstacles: typing.List[Polygon]) -> None:
    """ Plot the robot in the workspace.

    Parameters
    ----------
    robot : typing.Dict[str, typing.List[float]]
        A dictionary containing the robot's parameters.
    q : typing.List[float]
        A 2-element list representing the configuration of the robot
    obstacles : typing.List[Polygon]
        A list of polygons representing the obstacles
    """
    
    """
    example robot
    {'link1': [[-1.2, 0.5], [-1.2, -0.5], [2.3, -0.4], [2.3, 0.4], [-1.2, 0.5]],
    'link2': [[-0.3, 0.4], [-0.3, -0.4], [2.7, -0.2], [2.7, 0.2], [-0.3, 0.4]],
    'pivot1': [6.4, 2.5],
    'pivot2': [2.1, 0]}
    """
    ### Insert your code below:
    shape1, shape2, pivot1, pivot2 = q2poly(robot, q)
    
    plot_obstacles_robot(obstacles=obstacles, link1=shape1, link2=shape2, origin1=pivot1, origin2=pivot2)
 