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
    return np.array([[0, -V[2], V[1]],
                     [V[2], 0, -V[0]],
                     [-V[1], V[0], 0]])


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
    rotation_matrix = np.array([[a ** 2 + b ** 2 - c ** 2 - d ** 2, 2 * (b * c - a * d), 2 * (b * d + a * c)],
                                [2 * (b * c + a * d), a ** 2 + c ** 2 - b ** 2 - d ** 2, 2 * (c * d - a * b)],
                                [2 * (b * d - a * c), 2 * (c * d + a * b), a ** 2 + d ** 2 - b ** 2 - c ** 2]])

    return rotation_matrix


def invhat(khat):
    return np.array([(-khat[1, 2] + khat[2, 1]), (khat[0, 2] - khat[2, 0]), (-khat[0, 1] + khat[1, 0])]) / 2


def R2rot(R):
    """
    Recover k and theta from a 3 x 3 rotation matrix

        sin(theta) = | R-R^T |/2
        cos(theta) = (tr(R)-1)/2
        k = invhat(R-R^T)/(2*sin(theta))
        theta = atan2(sin(theta),cos(theta)

    :type    R: numpy.array
    :param   R: 3 x 3 rotation matrix
    :rtype:  (numpy.array, number)
    :return: ( 3 x 1 k unit vector, rotation about k in radians)

    """

    R1 = R - R.transpose()

    sin_theta = np.linalg.norm(R1) / np.sqrt(8)

    cos_theta = (np.trace(R) - 1.0) / 2.0
    theta = np.arctan2(sin_theta, cos_theta)

    # Avoid numerical singularity
    if sin_theta < 1e-6:

        if cos_theta > 0:
            return [0, 0, 1], 0
        else:
            B = (1.0 / 2.0) * (R + np.eye(3))
            k = np.sqrt([B[0, 0], B[1, 1], B[2, 2]])
            if np.abs(k[0]) > 1e-6:
                k[1] = k[1] * np.sign(B[0, 1] / k[0])
                k[2] = k[2] * np.sign(B[0, 2] / k[0])
            elif np.abs(k[1]) > 1e-6:
                k[2] = k[2] * np.sign(B[0, 2] / k[1])
            return k, np.pi

    k = invhat(R1) / (2.0 * sin_theta)
    return k, theta

def R2rpy(R):
    assert np.linalg.norm(R[0:2,0]) > np.finfo(float).eps * 10.0, "Singular rpy requested"
    
    r=np.arctan2(R[2,1],R[2,2])
    y=np.arctan2(R[1,0], R[0,0])
    p=np.arctan2(-R[2,0], np.linalg.norm(R[2,1:3]))
        
    return (r,p,y)    
