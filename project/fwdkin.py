import numpy as np
from utils import rodrigues_rotation, get_homogeneous
from Robot import Arm

def fwdkin(Arm, hlist, qlist):
    """Calculates the forward kinematics of the DofBot
    Args:
        Arm - Robot arm class with home config info and joint value
        hlist - the axes of rotation for each joint
        qlist - the angles of rotation for each joint
    Returns:
        End effector matrix (4x4) 
               
    """
    P = Arm.get_P()
    T = np.array(Arm.get_home_config())
    assert(len(hlist) == len(qlist), 
           "Number of joint axes and number of joints don't match")

    R_0T = np.eye(3)
    P_0T = P[:,-1]
    for i in range(len(hlist)-1, -1, -1):
        R_ij = rodrigues_rotation(hlist[:,i] * qlist[i])
        # R_01 @ R_12 @ R_23 @ R_34 @ R_45
        R_0T = R_ij @ R_0T
        P_0T = P[:,i] + R_ij @ P_0T

    return T @ get_homogeneous(R_0T, P_0T)

