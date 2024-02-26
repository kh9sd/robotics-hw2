import numpy as np

def generate_random_config(q_min: np.array, q_max: np.array) -> np.array:
    """_summary_

    Parameters
    ----------
    q_min : np.array
        1x4 numpy array of minimum angle for each joint
    q_max : np.array
        1x4 numpy array of maximum angle for each joint

    Returns
    -------
    np.array
        1 x 4 numpy array of joint angles, all within joint limits
    """
    return np.array([np.random.uniform(q_min[0], q_max[0]),
                     np.random.uniform(q_min[1], q_max[1]),
                     np.random.uniform(q_min[2], q_max[2]),
                     np.random.uniform(q_min[3], q_max[3])])

def M1(q_min: np.array, q_max: np.array, num_samples: int) -> np.array:
    """_summary_

    Parameters
    ----------
    q_min : np.array
        1x4 numpy array of minimum angle for each joint
    q_max : np.array
        1x4 numpy array of maximum angle for each joint
    num_samples : int
        number of samples to sample

    Returns
    -------
    np.array
        num_samples x 4 numpy array of joint angles, all within joint limits
    """

    result = np.zeros((num_samples, 4))

    for i in range(num_samples):
        result[i] = generate_random_config(q_min, q_max)
    
    return result

    