#!/usr/bin/env python3

## Header ##########################################################
import argparse
import matplotlib.pyplot as plt
import numpy as np
from structure import *
from solver import ActuationModel

## Response plot ###################################################
def plot_response(show=True):
    theta = 0.0  # initial orientation

    theta_list, I1_list, I2_list = [], [], []
    Ttotx_list, Ttoty_list, Ttotz_list = [], [], []
    Ftotx_list, Ftoty_list, Ftotz_list = [], [], []

    while theta <= 2 * np.pi:
        Structure1 = RobotDesign1(
            distance=2.5,
            moment_value=0.001875,
            number_magnets=3,
        )

        Eval = ActuationModel(
            structure=Structure1,
            position=np.array([[60.0], [60.0], [64.0]]),
            orientation=theta,
            desired_torque=np.array([[0.0], [0.0], [0.007]]),
            desired_force=np.array([[0.0], [0.0], [0.0]]),
        )

        theta_list.append(theta / np.pi * 180)
        I1_list.append(Eval.Iopt.flatten()[0])
        I2_list.append(Eval.Iopt.flatten()[1])
        Ttotx_list.append(Eval.Ttot.flatten()[0])
        Ttoty_list.append(Eval.Ttot.flatten()[1])
        Ttotz_list.append(Eval.Ttot.flatten()[2])
        Ftotx_list.append(Eval.Ftot.flatten()[0])
        Ftoty_list.append(Eval.Ftot.flatten()[1])
        Ftotz_list.append(Eval.Ftot.flatten()[2])

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
    plt.plot(theta_list, Ttotx_list, "r", label="Tm + Tf (x)", linewidth=2)
    plt.plot(theta_list, Ttoty_list, "b", label="Tm + Tf (y)", linewidth=2)
    plt.plot(theta_list, Ttotz_list, "k", label="Tm + Tf (z)", linewidth=2)
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 360, 45))
    plt.title("Torque Results")
    plt.xlabel("Theta (deg)")
    plt.ylabel("Torque (mN*m)")
    plt.grid(True)

    plt.figure(3)
    plt.plot(theta_list, Ftotx_list, "r", label="F (x)", linewidth=2)
    plt.plot(theta_list, Ftoty_list, "b", label="F (y)", linewidth=2)
    plt.plot(theta_list, Ftotz_list, "k", label="F (z)", linewidth=2)
    plt.xlim(0, 360)
    plt.xticks(np.arange(0, 360, 45))
    plt.title("Force Results")
    plt.xlabel("Theta (deg)")
    plt.ylabel("Force (mN)")
    plt.grid(True)

    if show:
        plt.show()

## Torque map ######################################################
def plot_torque_map(show=True):
    theta_range = range(0, 361)  # robot orientation
    delta_range = range(0, 361)  # input orientation

    Ttot_grid = np.zeros((len(theta_range), len(delta_range)))

    for i, theta_deg in enumerate(theta_range):
        theta_rad = np.deg2rad(theta_deg)

        for j, delta_deg in enumerate(delta_range):
            delta_rad = np.deg2rad(delta_deg)

            Structure2 = RobotDesign1(  # Giltinan et al.
                distance=0.1,
                moment_value=4.4e-12,
                number_magnets=5,
            )

            input_eval = ActuationModel(
                structure=Structure2,
                position=np.array([[60.0], [60.0], [55.0]]),
                orientation=delta_rad,
                # desired_Bz=10.0,
                desired_torque=np.array([[0.0], [0.0], [0.007]]),
                desired_force=np.array([[0.0], [0.0], [0.0]]),
            )

            real = ActuationModel(
                structure=Structure2,
                position=np.array([[60.0], [60.0], [55.0]]),
                orientation=theta_rad,
                # desired_Bz=10.0,
                desired_torque=np.array([[0.0], [0.0], [0.007]]),
                desired_force=np.array([[0.0], [0.0], [0.0]]),
                current_vector=input_eval.Iopt,
            )

            Ttot_grid[i, j] = real.Ttot.flatten()[2]

    theta_grid, delta_grid = np.meshgrid(delta_range, theta_range)

    plt.figure(figsize=(10, 8))
    contour = plt.contourf(
        delta_grid,
        theta_grid,
        Ttot_grid,
        cmap="viridis",
        levels=np.linspace(np.min(Ttot_grid), np.max(Ttot_grid), 50),
    )

    plt.colorbar(contour, label="Generated Torque (Tz)")
    plt.contour(delta_grid, theta_grid, Ttot_grid, levels=[0], colors="red", linewidths=1.5)
    plt.xlabel("Input orientation (deg)")
    plt.ylabel("Robot orientation (deg)")
    plt.title("Contour Plot of Generated Torque (Tz)")
    plt.xticks(np.arange(0, 361, 45))
    plt.yticks(np.arange(0, 361, 45))
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()

    if show:
        plt.show()

## Execution #######################################################
def main():
    parser = argparse.ArgumentParser(
        description="Plot response curves or torque map for the magnetic actuation model."
    )
    parser.add_argument(
        "--mode",
        choices=["response", "torque_map", "both"],
        default="response",
        help="Select which analysis to run.",
    )
    args = parser.parse_args()

    if args.mode == "both":
        plot_response(show=False)
        plot_torque_map(show=False)
        plt.show()
        return

    if args.mode == "response":
        plot_response()
        return

    plot_torque_map()


if __name__ == "__main__":
    main()
