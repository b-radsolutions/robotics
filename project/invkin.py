import numpy as np
from utils import R2rpy
from Robot import UR5Arm
from fwdkin import fwdkin


def invkin(arm: UR5Arm, Rd, Pd, q0, max_iters=2000, alpha=0.1, tol=1e-4):
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
    assert (q0.shape == (5, 1))
    q_prev = q0
    q_cur = q0

    iter = 0
    converged = np.array([False] * 5)
    while iter < max_iters and not converged.all():

        # get the homogenous transform of current q
        R_cur, P_cur = fwdkin(arm, q_cur)
        # get the postion and Orientation

        if iter == 0: print(R_cur, P_cur)

        P_err = P_cur - Pd
        R_err = R_cur @ Rd.T
        # calculate r
        r = np.array(R2rpy(R_err))

        J0T = arm.jacobian(q_cur)
        # computer the update value
        err = np.concatenate((r.T, P_err))

        converged = np.absolute(err) <= tol
        if converged.all():
            print(err)
            print(f"Inverse Kinematics Converged after {iter} iterations")
            return True, q_cur

        j = np.linalg.pinv(J0T) @ err
        q_cur = q_prev - alpha * j.reshape(5, 1)
        q_prev = q_cur

        iter += 1

    print("Inverse Kinematics Didn't Converge")
    return False, q_cur
