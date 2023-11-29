from fwdkin import fwdkin
from Robot import UR5Arm
import numpy as np
import quadprog

def path_plan(arm, q_0, N, P_dest, R_dest, K_r, K_p, E_r, E_p):
    lambda_ = 1 / (N - 1)
    assert 0 < lambda_ < 1
    P_0T, R_0T = fwdkin(arm, q_0)
    P_d_lambda_ = (1- lambda_)*P_0T + lambda_*P_dest
    P_d_lambda_prime = P_0T - P_dest
    # [k_1, theata] = Robotics Toolbox




    for i in range(N):
        L = pass
        U = pass
