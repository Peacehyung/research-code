#!/usr/bin/env python3

## Header #########################################################################################
import argparse
import matplotlib.pyplot as plt
import numpy as np
from structure import *
from solver import ActuationModel

## Plotting actuation response WRT input 6-DoF orientation ########################################
def plot_response():
    theta = 0.0                                                               # initial orientation

    theta_list, I1_list, I2_list = [], [], []
    tx_list, ty_list, tz_list = [], [], []
    fx_list, fy_list, fz_list = [], [], []

    while theta <= 2 * np.pi:

        Robot = RobotDesign(
            magnet_distance=2.5,
            moment_value=0.001875,
            number_magnets=3,
            pattern="e5",
        )

        Act = ActuationModel(
            robot_design=Robot,
            robot_position=np.array([[60.0], [60.0], [64.0]]),
            robot_orientation=theta,
            desired_torque=np.array([[0.0], [0.0], [0.007]]),
            desired_force=np.array([[0.0], [0.0], [0.0]]),
        )

        Act.compute_Response()

        theta_list.append(theta / np.pi * 180)
        I1_list.append(Act.CURR_VEC.flatten()[0])
        I2_list.append(Act.CURR_VEC.flatten()[1])
        tx_list.append(Act.T.flatten()[0])
        ty_list.append(Act.T.flatten()[1])
        tz_list.append(Act.T.flatten()[2])
        fx_list.append(Act.F.flatten()[0])
        fy_list.append(Act.F.flatten()[1])
        fz_list.append(Act.F.flatten()[2])

        theta += np.pi / 180

    plt.figure(1)
    plt.plot(theta_list, I1_list, "r", label="current input 1", linewidth=2)
    plt.plot(theta_list, I2_list, "b", label="current input 2", linewidth=2)
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 360, 45))
    plt.title("Current Input")
    plt.xlabel("Theta (deg)")
    plt.ylabel("Current (A)")
    plt.grid(True)

    plt.figure(2)
    plt.plot(theta_list, tx_list, "r", label="Tm + Tf (x)", linewidth=2)
    plt.plot(theta_list, ty_list, "b", label="Tm + Tf (y)", linewidth=2)
    plt.plot(theta_list, tz_list, "k", label="Tm + Tf (z)", linewidth=2)
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 360, 45))
    plt.title("Torque Results")
    plt.xlabel("Theta (deg)")
    plt.ylabel("Torque (mN*m)")
    plt.grid(True)

    plt.figure(3)
    plt.plot(theta_list, fx_list, "r", label="F (x)", linewidth=2)
    plt.plot(theta_list, fy_list, "b", label="F (y)", linewidth=2)
    plt.plot(theta_list, fz_list, "k", label="F (z)", linewidth=2)
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 360, 45))
    plt.title("Force Results")
    plt.xlabel("Theta (deg)")
    plt.ylabel("Force (mN)")
    plt.grid(True)

    plt.show()

## Torque map #####################################################################################
def plot_torque_map():
    theta_rng = range(0, 361)                                                   # robot orientation
    delta_rng = range(0, 361)                                                   # input orientation

    T_GRID = np.zeros((len(theta_rng), len(delta_rng)))

    for i, theta_deg in enumerate(theta_rng):
        theta_rad = np.deg2rad(theta_deg)

        for j, delta_deg in enumerate(delta_rng):
            delta_rad = np.deg2rad(delta_deg)

            Robot = RobotDesign(  # Giltinan et al.
                magnet_distance=0.1,
                moment_value=4.4e-12,
                number_magnets=5,
                pattern="e5",
            )

            InputAct = ActuationModel(
                robot_design=Robot,
                robot_position=np.array([[60.0], [60.0], [64.0]]),
                robot_orientation=delta_rad,
                desired_torque=np.array([[0.0], [0.0], [0.007]]),
                desired_force=np.array([[0.0], [0.0], [0.0]]),
            )            

            RobotAct = ActuationModel(
                robot_design=Robot,
                robot_position=np.array([[60.0], [60.0], [64.0]]),
                robot_orientation=theta_rad,
                desired_torque=np.array([[0.0], [0.0], [0.007]]),
                desired_force=np.array([[0.0], [0.0], [0.0]]),                
            )

            RobotAct.compute_Response(current_vector=InputAct.CURR_VEC)

            T_GRID[i, j] = RobotAct.T.flatten()[2]

    theta_grid, delta_grid = np.meshgrid(delta_rng, theta_rng)

    plt.figure(figsize=(10, 8))
    contour = plt.contourf(
        delta_grid,
        theta_grid,
        T_GRID,
        cmap="viridis",
        levels=np.linspace(np.min(T_GRID), np.max(T_GRID), 50),
    )

    plt.colorbar(contour, label="Generated Torque (Tz)")
    plt.contour(delta_grid, theta_grid, T_GRID, levels=[0], colors="red", linewidths=1.5)
    plt.xlabel("Input orientation (deg)")
    plt.ylabel("Robot orientation (deg)")
    plt.title("Contour Plot of Generated Torque (Tz)")
    plt.xticks(np.arange(0, 361, 45))
    plt.yticks(np.arange(0, 361, 45))
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()

    plt.show()

## Main function ##################################################################################
def main():
    # plot_response()
    plot_torque_map()        

## Execution ######################################################################################
if __name__ == "__main__":
    main()