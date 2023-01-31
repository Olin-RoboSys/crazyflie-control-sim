"""
Class for plotting a quadrotor

Original Author: Daniel Ingram (daniel-s-ingram)

Heavily modified by Kene Mbanisi
"""


import matplotlib.pyplot as plt
import control as ct
import numpy as np
from utils import SimData


class Quadrotor1D():
    """
    This class handles the plotting and evaluation of the crazyflie simulation

    You do not need to edit this class!
    """
    def __init__(self, init_state, pid_gains, cfparams, size=0.25, time_delta=0, show_animation=True):
        self.p1 = np.array([size / 2, 0])
        self.p2 = np.array([-size / 2, 0])

        self.x_data = []
        self.z_data = []
        self.show_animation = show_animation
        self.pid_gains = pid_gains
        self.cfparams = cfparams

        self.sim_data = SimData()

        self.time_delta = time_delta
        self.itx = 0

        if self.show_animation:
            plt.ion()
            self.fig = plt.figure()
            # for stopping simulation with the esc key.
            self.fig.canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            self.sub1 =  self.fig.add_subplot(1,2,1)
            self.sub2 =  self.fig.add_subplot(4,2,2)
            self.sub3 =  self.fig.add_subplot(4,2,4)
            self.sub4 =  self.fig.add_subplot(4,2,6)
            self.sub5 =  self.fig.add_subplot(4,2,8)

            self.fig.set_size_inches(12, 8)    

            self.fig.suptitle("1D Quadrotor Altitude Control", fontsize=16)


        self.update_plot(init_state, init_state, 0, 0)

    def update_plot(self, state, set_point, z_pos_raw, U):
        self.x = 0.
        self.z = state.z_pos
        self.set_point_z = set_point.z_pos
        self.sim_data.x_pos.append(0)
        self.sim_data.z_pos.append(state.z_pos)
        self.sim_data.z_vel.append(state.z_vel)
        self.sim_data.U.append(U)
        self.sim_data.U_clamped.append(np.clip(U, self.cfparams.minT, self.cfparams.maxT))
        self.sim_data.z_pos_raw.append(z_pos_raw)
        self.sim_data.z_pos_error.append(set_point.z_pos - state.z_pos)
        self.itx += 1

        self.z_vel_raw = self.diff(self.sim_data.z_pos_raw)
        

        if self.show_animation:
            self.plot()

    def plot(self):
        ### simulation
        num_index = len(self.sim_data.z_pos)
        t = np.linspace(0, num_index*self.time_delta, num_index)

        p1_t = self.p1 + np.array([self.x, self.z])
        p2_t = self.p2 + np.array([self.x, self.z])

        self.sub1.cla()
        self.sub2.cla()
        self.sub3.cla()
        self.sub4.cla()
        self.sub5.cla()

        # plot set point
        self.sub1.plot(self.x, self.set_point_z, 'go')

        # plot rotors
        self.sub1.plot([p1_t[0], p2_t[0]],
                     [p1_t[1], p2_t[1]], 'k.')

        # plot frame
        self.sub1.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]], 'r-')

        # plot track
        self.sub1.plot(self.sim_data.x_pos, self.sim_data.z_pos, 'b:')

        self.sub1.set_xlim(-5, 5)
        self.sub1.set_ylim(0, 15)
        self.sub1.set_xlabel('x [m]')
        self.sub1.set_ylabel('z [m]')
        self.sub1.set_title('Iteration: {}, Time: {:0.2f}'.format(self.itx, t[self.itx-1]))

        ### error plot
        self.sub2.plot(t, self.sim_data.z_pos_error, 'b')
        self.sub2.hlines(0, 0, t[self.itx-1], colors='black', linestyles='--')
        self.sub2.set_ylabel('error [m]')

        ### z position plot
        self.sub3.plot(t, self.sim_data.z_pos_raw, 'r-', label='raw meas')
        self.sub3.plot(t, self.sim_data.z_pos, 'b', label='filtered')
        self.sub3.hlines(self.set_point_z, 0, t[self.itx-1], colors='black', linestyles='--')
        self.sub3.set_ylabel('z_pos [m]')
        self.sub3.legend(fontsize=8)

        ### z velocity plot
        self.sub4.plot(t, self.z_vel_raw, 'r-', label='from raw meas')
        self.sub4.plot(t, self.sim_data.z_vel, 'b', label='filtered')
        self.sub4.set_ylabel('z_vel [m/s]')
        self.sub4.legend(fontsize=8)

        ### thrust plot
        self.sub5.plot(t, self.sim_data.U, 'b', label='commanded thrust')
        self.sub5.plot(t, self.sim_data.U_clamped, 'black', label='actual thrust')
        self.sub5.set_ylabel('Thrust [N]')
        self.sub5.legend(fontsize=8)
        
        plt.pause(0.075)

    def diff(self, array):
        init = np.array([0])
        if len(array) < 2:
            return init
        return np.append(init, np.diff(array)/self.time_delta )

    def evaluator(self):
        num_index = len(self.sim_data.z_pos)
        t = np.linspace(0, num_index*self.time_delta, num_index)
        S = ct.step_info(self.sim_data.z_pos, t)

        rise_time = S["RiseTime"]
        settling_time = S["SettlingTime"]
        overshoot = S["Overshoot"]
        steadystate_error = S["SteadyStateValue"] - self.set_point_z


        textstr = 'Control Performance for [Kp, Ki, Kd] = [ %.1f , %.1f , %.1f ]\n\
        ---------------------------------------------------------------------\n\
        Rise time             = %.3f [s]\n\
        Settling Time         = %.3f [s]\n\
        Overshoot             = %.3f [percent]\n\
        Steady State Error    = %.3f [m]\n' \
        %(self.pid_gains.kp, self.pid_gains.ki, self.pid_gains.kd, \
            rise_time, settling_time, overshoot, steadystate_error)

        plt.gcf().text(0.15, 0.01, textstr, fontsize=12)
        plt.subplots_adjust(bottom=0.25)

        plt.show(block=True)

