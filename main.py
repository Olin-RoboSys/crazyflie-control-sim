import sys

from QuadrotorSim import Quadrotor1D
from QuadrotorDynamics import QuadrotorDynamics1D
from Controller import Controller1D
from StateEstimator import StateEstimator1D
from utils import *




def run_sim(args):
    """Main function that runs the simulation"""
    
    # get/set simulation parameters
    pid_gains, sim_params = parse_args(args)
    sim_time = sim_params.sim_time
    time_delta = 0.05
    curr_time = 0
    thrust = 0

    # print information on cmd line
    print_info(sim_params)

    # get crazyflie params
    cfparams = CrazyflieParams()

    # set initial state
    state = State()
    state.z_pos = 0.0

    # instantiate dynamics object
    quad_dynamics = QuadrotorDynamics1D(state, cfparams,
                                        disturbance=sim_params.disturbance_flag)

    # instantiate controller object
    controller = Controller1D(cfparams, pid_gains)

    # instantiate state estimator object
    state_estimator = StateEstimator1D(cfparams, init_state=state)

    # instantiate quadrotor simulation object
    quad_sim = Quadrotor1D(state,
                           pid_gains,
                           cfparams,
                           size=0.75,
                           time_delta=time_delta,
                           show_animation=sim_params.show_animation_flag)

    # set the set_point/desired state
    set_point = State
    set_point.z_pos = 8.0
    set_point.z_vel = 0.0

    while curr_time < sim_time:

        # ------------------SENSE ----------------------------------------
        if sim_params.state_estimation_flag:
            # read sensor data
            z_pos_raw = quad_dynamics.TOF_sensor()
            # estimate state using kalman filter
            state = state_estimator.compute(z_pos_raw, thrust, time_delta)
        else:
            # read "fake perfect" state directly from dynamics
            state = quad_dynamics.fake_perfect_sensor()
        # ----------------------------------------------------------------


        # -------------- (THINK &) ACT -----------------------------------
        # compute thrust (i.e. u)
        thrust = controller.compute_commands(set_point, state)
        # ----------------------------------------------------------------

        # advance state using dynamics
        quad_dynamics.update(thrust, time_delta)

        # update plot
        if sim_params.state_estimation_flag:
            quad_sim.update_plot(state, set_point, z_pos_raw, thrust)
        else:
            quad_sim.update_plot(state, set_point, state.z_pos, thrust)
           
        curr_time += time_delta

    # evaluate the control performance
    quad_sim.evaluator()



if __name__ == "__main__":
    run_sim(sys.argv)
