"""
The quadrotor dynamics class handles the simulated dynamics 
and numerical integration of the system
"""

import random
import numpy as np
from scipy.integrate import odeint


class QuadrotorDynamics1D():
    """
    This class handles the simulation of crazyflie dynamics using an ODE solver

    You do not need to edit this class
    """
    def __init__(self, state, cfparams, disturbance=False):
        """
        Inputs:
        - state (State dataclass):              the current state of the system
        - cfparams (CrazyflieParams dataclass): model parameter class for the crazyflie
        - disturbance (bool):                   flag to include random force disturbance in the dynamics
        """
        self.state = state
        self.params = cfparams
        self.disturbance = disturbance
        self.disturbance_onset = 2.0 + float(random.randint(-2, 2)) # crude way of varying the disturbance onset (time)
        self.t = 0
        self.t0 = 0
        self.y0 = [self.state.z_pos, self.state.z_vel]

    def dynamic_model(self, y, t, F):
        """
        Function that represents the dynamic model of the 1D crazyflie

        Inputs:
        - y (list):     the current state of the system
        - t (float):    current simulation time
        - F (float):    total upward thrust

        Returns:
        - dydt (list):  the time derivative of the system state
        """
        # checks if and when to add a disturbance force
        if self.disturbance and (self.t > self.disturbance_onset):
            dydt = [y[1],
                    (1/self.params.mass) * (F - self.params.mass * self.params.g - \
                        np.random.normal(0.2, 1.0e-2))]   # disturbance force is modeled as normally distributed
        else:
            dydt = [y[1],
                    (1/self.params.mass) * (F - self.params.mass * self.params.g)]
        return dydt

    def update(self, F, time_delta):
        """
        Function advances the system state using an ODE solver

        Inputs:
        - F (float):            total upward thrust
        - time_delta (float):   discrete time interval for simulation
        """
        # enforce thrust limits
        F_clamped = np.clip(F, self.params.minT, self.params.maxT)

        # advance state using odeint
        ts = [self.t0, self.t0 + time_delta]
        y = odeint(self.dynamic_model, self.y0, ts, args=(F_clamped,))

        # update initial state
        self.y0 = y[1]
        self.t += time_delta

        self.state.z_pos = y[1][0]
        self.state.z_vel = y[1][1]

    def TOF_sensor(self):
        """
        Simulates a noisy time-of-flight sensor by adding Gaussian noise to true position value

        Returns:
        - (float):  noisy position state of the system
        """
        return self.state.z_pos + np.random.normal(0.0, 0.1) # true value + gaussian noise

    def fake_perfect_sensor(self):
        """
        Simulates a fake perfect sensor that reads the true full state (pos, vel) of the system

        Returns:
        - (State dataclass):  true state of the system
        """
        return self.state