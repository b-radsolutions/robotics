"""
def path_plan(arm, q_0, N, P_dest, R_dest, K_r, K_p, E_r, E_p):
    """"""Calculates path with quad-prog
        Args:
            arm - ROS NODE
            q_0 - inital position
            N - Number of steps
            P_dest - desired Position
            R_dest - desired Rotation
            K_r, K_p - positive feedback constants
            E_r, E_p - constants that weight relative
            importance of objective terms

        Returns:

        """"""
    lambda_ = 1 / (N - 1)
    assert 0 < lambda_ < 1
    P_0T, R_0T = fwdkin_no_homo(arm, q_0)

    # EE Position Path
    P_d_lambda_ = (1- lambda_)*P_0T + lambda_*P_dest
    # EE Path dPd(y)/dy
    P_d_lambda_prime = P_0T - P_dest

    # Rotation Error
    ER = R_0T * R_dest

    # Convert to K-thetha representation
    k_1, theata_0 = R2rot(E_r)

    #




    q = q_0
    for i in range(1, N):
        Lower_Bound = 0
        Upper_Bound = 0
        J = arm.jacobian(q)
        R, P = fwdkin_no_homo(arm, q)


        # Get Position and Rotation Velocity
        Vr =
        Vp =
"""