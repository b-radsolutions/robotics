import numpy as np
import utils

class UR5Arm:
    def __init__(self, P, H, limits=None):
        """
        Args: 
            P - 3 x N matrix where column i is the position of
                point i to point i+1. (N-1 is the number of joints)
            H - 3 x 5 matrix to denote the axis of rotation of each
                joint in the home configuration.
            limits - joint angle limits
        """
        self.P = P
        self.H = H
        self.L = limits

    def get_P(self):
        """Returns a 3x6 array whose columns contains
        the relative position of joint i to i+1
        """
        return self.P

    def get_H(self):
        """Returns a 3x5 array whose columns constain 
        the axes of rotation of each joint
        """
        return self.H

    def get_Limits(self)->list:
        """Returns the joint limits in a list of tuples length: 6 (including gripper)
        """
        return self.L
        
    def jacobian(self, joint_angles)->np.array:
        """Returns the jacobian of this robot arm with joint angles
        """
        pass
