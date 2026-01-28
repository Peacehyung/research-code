#!/usr/bin/env python3

## Header ##########################################################
import matplotlib.pyplot as plt
import numpy as np
from structure import *
from evaluation import Evaluation

## Main function ###################################################
theta = 0.0                                   # initial orientation

theta_list, I1_list, I2_list = [], [], []
Ttotx_list, Ttoty_list, Ttotz_list = [], [], []
Ftotx_list, Ftoty_list, Ftotz_list = [], [], []

def outputPlot():

    global theta  

    while theta <= 2*np.pi:

        Structure1 = Config1(
            distance=2.5, 
            moment_value=0.001875,
            number_magnets=3,
            )

        Eval = Evaluation(
            structure=Structure1,
            position=np.array([[60.0], [60.0], [64.0]]),
            orientation=theta,
            desired_torque=np.array([[0.0], [0.0], [0.007]]),
            desired_force=np.array([[0.0], [0.0], [0.0]]),
            )
        
        theta_list.append(theta/np.pi*180)
        I1_list.append(Eval.Iopt.flatten()[0])
        I2_list.append(Eval.Iopt.flatten()[1])
        Ttotx_list.append(Eval.Ttot.flatten()[0])
        Ttoty_list.append(Eval.Ttot.flatten()[1])
        Ttotz_list.append(Eval.Ttot.flatten()[2])
        Ftotx_list.append(Eval.Ftot.flatten()[0])
        Ftoty_list.append(Eval.Ftot.flatten()[1])
        Ftotz_list.append(Eval.Ftot.flatten()[2])

        theta += np.pi/180

    plt.figure(1)
    plt.plot(theta_list, I1_list, 'r', label='current input 1', linewidth=2)
    plt.plot(theta_list, I2_list, 'b', label='current input 2', linewidth=2)
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 360, 45))
    # plt.ylim(0.0, 0.0105)
    # plt.yticks(np.arange(0.002, 0.01, 0.002))
    plt.title("Current Input")
    plt.xlabel("Theta (deg)")
    plt.ylabel("Current (A)")
    plt.grid(True)
    # plt.legend()

    plt.figure(2)
    plt.plot(theta_list, Ttotx_list, 'r', label='Tm + Tf (x)', linewidth=2)
    plt.plot(theta_list, Ttoty_list, 'b', label='Tm + Tf (y)', linewidth=2)
    plt.plot(theta_list, Ttotz_list, 'k', label='Tm + Tf (z)', linewidth=2)
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 360, 45))
    # plt.ylim(0.0, 0.0105)
    # plt.yticks(np.arange(0.002, 0.01, 0.002))
    plt.title("Torque Results")
    plt.xlabel("Theta (deg)")
    plt.ylabel("Torque (mN*m)")
    plt.grid(True)
    # plt.legend()

    plt.figure(3)
    plt.plot(theta_list, Ftotx_list, 'r', label='F (x)', linewidth=2)
    plt.plot(theta_list, Ftoty_list, 'b', label='F (y)', linewidth=2)
    plt.plot(theta_list, Ftotz_list, 'k', label='F (z)', linewidth=2)
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 360, 45))
    # plt.ylim(0.0, 0.0105)
    # plt.yticks(np.arange(0.002, 0.01, 0.002))
    plt.title("Force Results")
    plt.xlabel("Theta (deg)")
    plt.ylabel("Force (mN)")
    plt.grid(True)
    # plt.legend()

    plt.show()

## Execution #######################################################
if __name__ == '__main__':
    outputPlot()
