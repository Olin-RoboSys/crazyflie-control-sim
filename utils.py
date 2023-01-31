""" catch-all script for utility data structures (structs) and functions """

from typing import List
from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class CrazyflieParams:
    """  
    This dataclass captures the basic parameters of the Crazyflie 2.0

    Credits: 2016 Bernd Pfrommer
    """

    I = np.array([[1.43e-5,   0,          0], 
                  [0,         1.43e-5,    0],
                  [0,         0,          2.89e-5]])
    invI = np.linalg.inv(I)

    mass:   float = 0.030
    I:      np.ndarray = I
    invI:   np.ndarray = invI
    g:      float = 9.81
    L:      float = 0.046
    max_angle: float = 40*math.pi/180
    maxT:   float = 2.5*mass*g
    minT:   float = 0.05*mass*g


@dataclass
class State:
    """This dataclass represents the system state (pos and vel) """
    z_pos: float = 0.0
    z_vel: float = 0.0


@dataclass
class SimData:
    """This dataclass captures relevant simulation data for ease of storage"""
    x_pos:          List[float] = field(default_factory=list)
    z_pos:          List[float] = field(default_factory=list)
    z_vel:          List[float] = field(default_factory=list)
    U :             List[float] = field(default_factory=list)
    U_clamped :     List[float] = field(default_factory=list)
    z_pos_raw :     List[float] = field(default_factory=list)
    z_pos_error :   List[float] = field(default_factory=list)


@dataclass
class PIDGains:
    """This dataclass captures the controller PID gains """
    kp: int = 0
    ki: int = 0
    kd: int = 0


@dataclass
class SimulationParameters:
    """This dataclass captures relevant simulation settings"""
    disturbance_flag: bool = False
    state_estimation_flag: bool = False
    show_animation_flag: bool = True
    sim_time: float = 10.0


def parse_args(args):
    """
    This is a NAIVE argument parser to help with passing values from the commandline.
    Please use carefully. The order matters when specifying parameters in terminal/cmd.

    Inputs:
    - args (list):   argument list from sys.argv

    Returns:
    - pid_gains (PIDGains dataclass):               pid gain values
    - sim_params (SimulationParameters dataclass)   simulation parameters
    """
    pid_gains = PIDGains()
    sim_params = SimulationParameters()

    print(args)

    if len(args) < 2:
        return pid_gains, sim_params

    if len(args) == 2:
        pid_gains.kp = float(args[1][3:])
    elif len(args) == 3:
        pid_gains.kp = float(args[1][3:])
        pid_gains.ki = float(args[2][3:])
    elif len(args) == 4:
        pid_gains.kp = float(args[1][3:])
        pid_gains.ki = float(args[2][3:])
        pid_gains.kd = float(args[3][3:])
    elif len(args) == 5:
        pid_gains.kp = float(args[1][3:])
        pid_gains.ki = float(args[2][3:])
        pid_gains.kd = float(args[3][3:])
        sim_params.sim_time = float(args[4][9:])
    elif len(args) == 6:
        pid_gains.kp = float(args[1][3:])
        pid_gains.ki = float(args[2][3:])
        pid_gains.kd = float(args[3][3:])
        sim_params.sim_time = float(args[4][9:])
        sim_params.disturbance_flag = True if args[5][17:] == 'True' else False
    elif len(args) == 7:
        pid_gains.kp = float(args[1][3:])
        pid_gains.ki = float(args[2][3:])
        pid_gains.kd = float(args[3][3:])
        sim_params.sim_time = float(args[4][9:])
        sim_params.disturbance_flag = True if args[5][17:] == 'True' else False
        sim_params.state_estimation_flag = True if args[6][22:] == 'True' else False
    elif len(args) == 8:
        pid_gains.kp = float(args[1][3:])
        pid_gains.ki = float(args[2][3:])
        pid_gains.kd = float(args[3][3:])
        sim_params.sim_time = float(args[4][9:])
        sim_params.disturbance_flag = True if args[5][17:] == 'True' else False
        sim_params.state_estimation_flag = True if args[6][22:] == 'True' else False
        sim_params.show_animation_flag = True if args[7][20:] == 'True' else False
    
    return pid_gains, sim_params


def print_info(sim_params):
    """
    Simple function to print some simulation info to the terminal/cmdline

    Inputs:
    - sim_params (SimulationParameters dataclass)   simulation parameters

    """
    intro_txt = \
    f'\n \
    Crazyflie 1D Altitude Control Exercise\n \
    =====================================================\n\n \
    Simulation settings: \n\n \
    1. Disturbance  =  {sim_params.disturbance_flag}\n \
    2. State estimator =  {sim_params.state_estimation_flag}\n \
    3. Show animation =  {sim_params.show_animation_flag} \n \
    4. Simulation time = {sim_params.sim_time} \n '
    print(intro_txt)