import numpy as np
from robot import Simple_Manipulator as Robot

def M5(robot: Robot, path: np.array) -> np.array:
    """Smooth the given path

    Parameters
    ----------
    robot : Robot
        our robot object
    path : np.array
        Nx4 numpy array containing a collision-free path between q_start and q_goal

    Returns
    -------
    np.array
        Nx4 numpy array containing a smoothed version of the
        input path, where some unnecessary intermediate
        waypoints may have been removed
    """
    config_lst = list(path)
    result_lst = []
    #student work start here
    
    result_lst.append(config_lst[-1])
    cur_ending_config_idx = len(config_lst) -1

    while cur_ending_config_idx != 0:
        cur_ending_config = config_lst[cur_ending_config_idx]
        for i, config in enumerate(config_lst):

            if robot.check_edge(config, cur_ending_config):
                result_lst.append(config)
                cur_ending_config_idx = i
                break

    result_lst.reverse()
    return np.array(result_lst)
