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
    return np.vstack((np.concatenate((R, T), axis=1), np.array([0, 0, 0, 1])))


def vec_to_so3(V):
    """Converts a vector to an so(3) representation
    """
    return np.array([[    0, -V[2],  V[1]],
                     [ V[2],     0, -V[0]],
                     [-V[1],  V[0],     0]])


def euler_rodrigues(axis, angle):
    """Returns a 3x3 matrix that represents a rotation 
    around the axis by theta
    Args: 
        axis: 3x1 defining axis rotation
        theta: angle of rotation

    """
    # Normalize the axis
    axis = (axis) / np.linalg.norm(axis)
    
    # Compute the necessary components
    a = np.cos(angle / 2.0)
    b, c, d = -axis * np.sin(angle / 2.0)
    
    # Compute the rotation matrix
    rotation_matrix = np.array([[a**2 + b**2 - c**2 - d**2, 2 * (b*c - a*d), 2 * (b*d + a*c)],
                                [2 * (b*c + a*d), a**2 + c**2 - b**2 - d**2, 2 * (c*d - a*b)],
                                [2 * (b*d - a*c), 2 * (c*d + a*b), a**2 + d**2 - b**2 - c**2]])
    
    return rotation_matrix


