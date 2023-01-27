import numpy as np

from utils import State

class StateEstimator1D():
    """
    This class estimates the state (position and velocity) of the system based 
    on noisy sensor measurements using a kalman filter

    You are to implement the "compute" method.
    """
    def __init__(self, params, init_state):
        """
        Inputs:
        - params (CrazyflieParams dataclass):       model parameter class for the crazyflie
        - init_state (State dataclass):             initial state of the system
        """

        # your code here

        pass
        

    def compute(self, z_meas, U, time_delta):
        """
        Estimates the current system state given the noisy measurement and control input

        Inputs:
        - z_meas (float):       current noisy measurement from the system sensor
        - U (float):            computed control input (thrust)
        - time_delta (float):   discrete time interval for simulation

        Returns:
        - filtered_state (State): estimated system state (z_pos, z_vel) using the kalman filter

        """
        filtered_state = State()

        # your code here

        return filtered_state
