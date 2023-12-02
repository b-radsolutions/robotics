import numpy as np
from utils import rot, get_homogeneous
from Robot import UR5Arm


def fwdkin(arm: UR5Arm, theta: np.array) -> tuple():
    """Calculates the forward kinematics of the DofBot
    Args:
        arm - Robot arm class with home config info and joint value
        theta - (5 x 1) the angles of rotation for each joint
    Returns:
        End effector matrix (4x4) 
    """
    P = arm.get_P()
    H = arm.get_H()

    R_0T = np.eye(3)
    P_0T = P[:, 0]

    for i in range(1, P.shape[1]):
        R_ij = rot(H[:, i - 1], theta[i - 1, 0])

        R_0T = R_0T @ R_ij
        P_0T = P_0T + R_0T @ P[:, i]
    H = get_homogeneous(R_0T, P_0T.reshape(3, 1))

    return H[0:3, 0:3], H[:3, 3]
