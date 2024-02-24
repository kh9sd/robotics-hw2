import numpy as np
from shapely.geometry import Polygon as Polygon_shapely
import typing
from math import cos, sin


def homo_2d_rot(theta):
    return np.array([[cos(theta), -sin(theta), 0],
                     [sin(theta), cos(theta), 0],
                     [0, 0, 1]])

def homo_2d_trans(translation):
    assert(len(translation) == 2)

    return np.array([[1, 0, translation[0]],
                     [0, 1, translation[1]],
                     [0, 0, 1]])

def apply_transform(homo_trans, pos: typing.List[float]) -> np.array:
    pos = pos + [1]
    numpyed_pos = np.array(pos)
    result = (homo_trans @ numpyed_pos)[:-1]
    return np.array(result)


def q2poly(robot: typing.Dict[str, typing.List[float]], q: typing.List[float]) -> typing.Tuple[np.array, np.array, np.array, np.array]:
    """ A function that takes in the robot's parameters and a configuration and 
    returns the vertices of the robot's links after transformation and the pivot points of the links after transformation

    Parameters
    ----------
    robot : typing.dict[str, typing.List[float]]
        A dictionary containing the robot's parameters
    q : typing.List[float]
        A 2-element list representing the configuration of the robot
        The configuration of the arm is given by q = (q1, q2), where q1 is the angle between x0 and
        x1, and q2 is the angle between x1 and x2

    Returns
    -------
    typing.Tuple[np.array, np.array, np.array, np.array]
        np.array: 
            a numpy array representing the vertices of the first link of the robot after transformation
        np.array: 
            a numpy array representing the vertices of the second link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the first link of the robot after transformation
        np.array: 
            a numpy array representing the pivot point of the second link of the robot after transformation
    """

    
    ### Insert your code below: ###

    q_1_to_base, q_2_to_1 = q

    first_link_homo_trans = homo_2d_trans([6.4,2.5]) @ homo_2d_rot(q_1_to_base)
    second_link_homo_trans = first_link_homo_trans @ homo_2d_trans([2.1,0]) @ homo_2d_rot(q_2_to_1)

    # for some goddamnit reason the first vertix is "looped"
    robot_link1_vertices = robot["link1"][:-1]
    shape1 = np.array([apply_transform(first_link_homo_trans, vertex) for vertex in robot_link1_vertices])

    robot_link2_vertices = robot["link2"][:-1]
    shape2 = np.array([apply_transform(second_link_homo_trans, vertex) for vertex in robot_link2_vertices])

    pivot1 = np.array(apply_transform(first_link_homo_trans, [0, 0]))
    pivot2 = np.array(apply_transform(second_link_homo_trans, [0, 0]))

    return shape1, shape2, pivot1, pivot2
