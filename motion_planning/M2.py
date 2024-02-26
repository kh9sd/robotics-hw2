import typing
import numpy as np
from networkx import Graph
from robot import Simple_Manipulator as Robot

import math


def generate_random_config(q_min: np.array, q_max: np.array) -> typing.List:
    """_summary_

    Parameters
    ----------
    q_min : np.array
        1x4 numpy array of minimum angle for each joint
    q_max : np.array
        1x4 numpy array of maximum angle for each joint

    Returns
    -------
    list of length 4 of joint angles, all within joint limits
    """
    return [np.random.uniform(q_min[0], q_max[0]),
            np.random.uniform(q_min[1], q_max[1]),
            np.random.uniform(q_min[2], q_max[2]),
            np.random.uniform(q_min[3], q_max[3])]


def M2(robot: Robot, num_samples: int, num_neighbors: int) -> typing.Tuple[np.array, Graph]:
    """ Implement the PRM algorithm

    Parameters
    ----------
    robot : Robot
        our pybullet robot class
    num_samples : int
        number of samples in PRM
    num_neighbors : int
        number of closest neighbors to consider in PRM

    Returns
    -------
    typing.Tuple[np.array, Graph]
        np.array: 
            num_samples x 4 numpy array, sampled configurations in the roadmap (vertices)
        G: 
            a NetworkX graph object with weighted edges indicating the distance between connected nodes in the joint configuration space.
            This should be impelemented as an undirected graph.
    """

    # HINTS
    # useful functions and parameters
    # robot.lower_lims, robot.upper_lims -> Joint Limits
    # robot.check_edge() -> check the linear path between 2 joint configurations for collisions

    samples_lst = []
    while len(samples_lst) != num_samples:
        cur_sample = generate_random_config(robot.lower_lims, robot.upper_lims)
        if (not robot.is_in_collision(cur_sample)):
            samples_lst.append(cur_sample)

    G = Graph()
    for sample in samples_lst:
        sorted_samples_by_dist = sorted(samples_lst, key=lambda other_sample: math.dist(other_sample, sample))

        for other_sample in sorted_samples_by_dist[1:1+num_neighbors]: # ignore first one, its just the same node (distance 0)
            # Returns True, if edge is collision free
            if robot.check_edge(sample, other_sample):
                G.add_edge(tuple(sample), tuple(other_sample), weight=math.dist(other_sample, sample))

    samples = np.array(samples_lst)
    assert samples.shape == (num_samples, 4)

    ### student code start here
    # raise NotImplementedError
    
    return samples, G
