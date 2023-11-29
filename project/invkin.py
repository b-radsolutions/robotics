import numpy as np
from utils import euler_rodrigues, get_homogeneous
from Robot import UR5Arm, jacobian
from fwdkin import fwdkin

def invkin(arm: UR5Arm, Rd, Pd, q0, max_iters=200, Kp = 0.3*np.eye(3), KR = 0.3*np.eye(3), tol = 1e-4):
    """Calculates the inverse kinematics of a 
    Args:
        Arm - Robot arm class with end effector home config info and 
            current joint values
        Rd - the desired Rotation of EE
        Pd - the desired Positon of EE
        q0 - (5x1) the initial guess joint configuration
        max_iters - maximum number of gradient steps
        tol - convergence criteria
    """
    q_cur = q0
    
    iter = 0
    converged = False
    while iter < max_iters and not converged:

        # get the homogenous transform of current q
        H_cur = fwdkin(Arm, hlist, q_cur)
        # get the postion and Orientation
        R_cur = H[0:3, 0:3]
        P_cur = H[:3, 3]
        
        P_err = np.linalg.norm(P-Pd)
        R_err = R @ Rd.T

        J0T = arm.jacobian(q_cur) 
        
        
        
