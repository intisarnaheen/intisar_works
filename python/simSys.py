import numpy as np
from math import pi
from my_ctl import *
from my_taskSpace_ctl import *
from matplotlib import pyplot as plt
import time


def simSys(robot, dt, nSteps, ctls, target, pauseTime):

    if 'pauseTime' not in locals() or not pauseTime:
        pauseTime = False

    states = np.zeros((nSteps * len(ctls), robot.dimState))
    states[:: nSteps, ::2] = np.tile([-pi, 0], (len(ctls), 1))  # repmat([-pi, 0], length(ctls), 1);

    plt.ion()
    if pauseTime or target['cartCtl']:
        fig = plt.figure()
        ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2.5, 2.5), ylim=(-2.5, 2.5))
        plt.plot(2 * np.array([-1.1, 1.1]), np.array([0, 0]), 'b--')
        line, = ax.plot([], [], 'o-', lw=2, color='k' ,markerfacecolor='w', markersize=12)
        plt.xlabel('x-axis [m]', fontsize=15)
        plt.ylabel('y-axis [m]', fontsize=15)

        plt.draw()
        robot.visualize(states[0, :], line)

    for k in range(len(ctls)):

        for i in range((nSteps - 1)):

            c_idx = i + k * nSteps

            gravity, coriolis, M = robot.getDynamicsMatrices(states[c_idx, :])

            if not target['cartCtl']:
                u = my_ctl(ctls[k], states[c_idx, ::2], states[c_idx, 1::2], target['q'][i, :], target['qd'][i, :],
                           target['qdd'][i, :], states[:c_idx, ::2], target['q'][:i, :], gravity, coriolis, M)
            else:
                J, cart = robot.getJacobian(np.mat(states[c_idx, ::2]), 2)
                u = my_taskSpace_ctl(ctls[k], dt, np.mat(states[c_idx, ::2]).transpose(),
                                     np.mat(states[c_idx,1::2]).transpose(), np.mat(gravity).transpose(),
                                     np.mat(coriolis).transpose(), M, np.mat(J),
                                     np.mat(cart).transpose(),  np.mat(target['x'][i,:]).transpose())

            qdd = M ** -1 * (u - np.mat(coriolis).transpose() - np.mat(gravity).transpose())
            states[c_idx + 1, 1::2] = states[c_idx, 1::2] + dt * qdd.transpose()
            states[c_idx + 1, ::2] = states[c_idx, 0::2] + dt * states[c_idx + 1, 1::2]

            if pauseTime:
                robot.visualize(states[i + 1, :], line)
                plt.pause(pauseTime)

        if target['cartCtl']:
            robot.visualize(states[0,:], line)

            robot.visualize(states[-1,:], line)

            plt.pause(5)

    return states
