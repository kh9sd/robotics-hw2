from networkx import Graph, shortest_path
import numpy as np
from robot import Simple_Manipulator as Robot
import typing

import math
from collections import defaultdict
import heapq


def M3(robot: Robot, samples: np.array, graph: Graph, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """ Find a path from q_start to q_goal using the PRM roadmap

    Parameters
    ----------
    robot : Robot
        our robot object
    samples : np.array
        num_samples x 4 numpy array of nodes/vertices in the roadmap
    G : Graph
        An undirected NetworkX graph object with the number of nodes equal to num_samples, 
        and weighted edges indicating collision free connections in the robot's configuration space
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
    distances = defaultdict(lambda : math.inf)
    prev = {}

    # link up start and goal
    for sample in [list(q_start), list(q_goal)]:
        for other_sample in samples: 
            # Returns True, if edge is collision free
            if robot.check_edge(sample, other_sample):
                graph.add_edge(tuple(sample), tuple(other_sample), weight=math.dist(other_sample, sample))
    
    distances[tuple(q_start)] = 0
    todo_queue = [(0, tuple(q_start))]

    while todo_queue:
        cur_d, cur_node = heapq.heappop(todo_queue)

        for neighbor, datadict in graph.adj[cur_node].items():
            candidate_dist = cur_d + datadict["weight"]

            if candidate_dist < distances[neighbor]:
                distances[neighbor] = candidate_dist
                prev[neighbor] = cur_node
                heapq.heappush(todo_queue, (candidate_dist, neighbor))

    if prev.get(tuple(q_goal)) is None:
        return np.zeros((1,4)), False
    
    cur_node = tuple(q_goal)
    path_lst = []

    while(cur_node is not None):
        path_lst.append(cur_node)
        cur_node = prev.get(cur_node)
    
    path_lst.reverse()

    return np.array(path_lst), True

