from M1 import M1
import numpy as np
from robot import Simple_Manipulator as Robot
import typing

import math
import random
from networkx import Graph
import networkx
import M2

def getClosestNode(graph: Graph, cur_sample: typing.Tuple[float, float, float, float]) -> typing.Tuple[float, float, float, float]:
    return min((node for node in graph), key=lambda n: math.dist(cur_sample, n))


def nextConfig(closest_vertex: typing.Tuple[float, float, float, float], cur_sample: typing.Tuple[float, float, float, float], max_step: float) -> typing.Tuple[float, float, float, float]:
    if math.dist(cur_sample, closest_vertex) > max_step:
        from_v = np.array(closest_vertex)
        to_v = np.array(cur_sample)
        change_v = to_v - from_v
        norm_change_v = change_v / np.linalg.norm(change_v)

        return tuple(from_v + (norm_change_v * max_step))
    else:
        return cur_sample


def rrt_random_sample(robot, q_goal: np.array) -> typing.Tuple[float, float, float, float]:
    if random.randrange(0, 10) == 0:
        return tuple(q_goal)
    else:
        return tuple(M2.generate_random_config(robot.lower_lims, robot.upper_lims))


def M4(robot: Robot, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """Implement RRT algorithm to find a path from q_start to q_goal

    Parameters
    ----------
    robot : Robot
        our robot object
    q_start : np.array
        1x4 numpy array denoting the start configuration
    q_goal : np.array
        1x4 numpy array denoting the goal configuration

    Returns
    -------
    typing.Tuple[np.array, bool]
        np.array:
            Nx4 numpy array containing a collision-free path between
            q_start and q_goal, if a path is found. The first row
            should be q_start, the final row should be q_goal.
        bool:
            Boolean denoting whether a path was found
    """

    MAX_ITERATIONS = 1000
    MAX_STEP_SIZE = 1
    graph = Graph()
    graph.add_node(tuple(q_start))

    for _ in range(MAX_ITERATIONS):
        cur_sample = rrt_random_sample(robot, q_goal)
        closest_vertex = getClosestNode(graph, cur_sample)
        next_config = nextConfig(closest_vertex, cur_sample, MAX_STEP_SIZE)

        if robot.check_edge(np.array(closest_vertex), np.array(next_config)):
            graph.add_edge(closest_vertex, next_config)

    try:
        path = networkx.shortest_path(graph, tuple(q_start), tuple(q_goal))
        return np.array(path), True
    except networkx.exception.NodeNotFound:
        return np.zeros((1,4)), False
