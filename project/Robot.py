import numpy as np

class Arm():
    def __init__(self, home, P):
        """
        Args: 
            home - 4 x 4 matrix. The home orientation and position of the end effector
            P - 3 x N matrix where column i is the position of
                point i to point i+1. (N-1 is the number of joints)
        """
        self.Z = home
        self.P = positions 

    def get_home():
        return self.Z

    def get_postions():
        return self.P
    
