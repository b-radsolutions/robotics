import numpy as np

def get_homogeneous(R, T):
    """returns the homogeneous representation (H) given R and T
    Args: 
        R (3x3)
        T (3x1)
    Returns:
        H (4 x 4)

                  |  
              R   | T
                  |
            - - -   - 
            0 0 0   1
    """
    return np.vstack(np.column_stack(R, T), np.array([0, 0, 0, 1]))


def vec_to_so3(V):
    """Converts a vector to an so(3) representation
    """
    return np.array([[    0, -V[2],  V[1]],
                     [ V[2],     0, -V[0]],
                     [-V[1],  V[0],     0]])


def rodrigues_rotation(axis, theta):
    """Returns a 3x3 matrix that represents a rotation 
    around the axis by theta
    Args: 
        axis: 3x1 defining axis rotation
        theta: angle of rotation

    """
    axis = axis/np.linalg.norm(axis)
    axis_cross = vec_to_so3(axis)
    
    # Rodrigues' Formula
    R = np.eye(3) * np.cos(theta) + np.sin(theta) * axis_cross + (1 - np.cos(theta)) * np.outer(axis, axis)
    return R


