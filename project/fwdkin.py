import numpy as np
from utils import euler_rodrigues, get_homogeneous
from Robot import UR5Arm

def fwdkin(arm: UR5Arm, theta: np.array)->tuple():
    """Calculates the forward kinematics of the DofBot
    Args:
        arm - Robot arm class with home config info and joint value
        theta - the angles of rotation for each joint
    Returns:
        End effector matrix (4x4) 
    """
    P = arm.get_P()
    H = arm.get_H()
    
    R_0T = np.eye(3)
    P_0T = P[:,-1]
    
    for i in range(np.shape(H)[1]-1, -1, -1):
        R_ij = euler_rodrigues(H[:,i],theta[i])
        # R_01 @ R_12 @ R_23 @ R_34 @ R_45
        R_0T = R_ij @ R_0T
        P_0T = P[:,i] + R_ij @ P_0T

    return get_homogeneous(R_0T, P_0T.reshape(3,1))
