import numpy as np

def M0() -> np.array:
    """ Test different configuration of the robot. 
    Play around with different joint angles to see how the robot moves

    Returns
    -------
    np.array
        1x4 numpy array of joint angles
    """
    # return  np.array([np.pi/4, 0, 0, 0])
    # return  np.array([0, np.pi/4, 0, 0])
    # return  np.array([0, 0, np.pi/4, 0])
    # return  np.array([0, 0, 0, np.pi/4])

    # return  np.array([np.pi/2, np.pi/4, 0, 0])
    # return  np.array([0, np.pi/4, np.pi/2, 0])

    # return  np.array([np.pi/4, np.pi/4, np.pi/4, 0])

    return  np.array([0, -np.pi/4, 0, -np.pi/4])
