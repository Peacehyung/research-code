#!/usr/bin/env python3

## Header ##########################################################
import matplotlib.pyplot as plt
import numpy as np
from structure import *
from evaluation import Evaluation

## Main function ###################################################
def main():
    theta_range = range(0, 361)                  # robot orientation
    delta_range = range(0, 361)                  # input orientation

    Ttot_grid = np.zeros((len(theta_range), len(delta_range)))

    for i, theta_deg in enumerate(theta_range):  
        theta_rad = np.deg2rad(theta_deg)

        for j, delta_deg in enumerate(delta_range):
            delta_rad = np.deg2rad(delta_deg)

            Structure2 = Config1(                  # Giltinan et al.
                distance=0.1,
                moment_value=4.4e-12,
                number_magnets=5,
            )

            input = Evaluation(
                structure=Structure2,
                position=np.array([[60.0], [60.0], [55.0]]),
                orientation=delta_rad,
                # desired_Bz=10.0,
                desired_torque=np.array([[0.0], [0.0], [0.007]]),
                desired_force=np.array([[0.0], [0.0], [0.0]])
            )

            real = Evaluation(
                structure=Structure2,
                position=np.array([[60.0], [60.0], [55.0]]),
                orientation=theta_rad,
                # desired_Bz=10.0,
                desired_torque=np.array([[0.0], [0.0], [0.007]]),
                desired_force=np.array([[0.0], [0.0], [0.0]]),
                current_vector=input.Iopt
            )

            Ttot_grid[i, j] = real.Ttot.flatten()[2]

    Ttot_grid = np.array(Ttot_grid)
    theta_grid, delta_grid = np.meshgrid(delta_range, theta_range)

    plt.figure(figsize=(10, 8))
    contour = plt.contourf(delta_grid, theta_grid, Ttot_grid,
                           cmap='viridis',
                           levels=np.linspace(np.min(Ttot_grid), np.max(Ttot_grid), 50))

    plt.colorbar(contour, label='Generated Torque (Tz)')
    plt.contour(delta_grid, theta_grid, Ttot_grid, levels=[0], colors='red', linewidths=1.5)
    plt.xlabel("Input orientation (deg)")
    plt.ylabel("Robot orientation (deg)")
    plt.title("Contour Plot of Generated Torque (Tz)")
    plt.xticks(np.arange(0, 361, 45))
    plt.yticks(np.arange(0, 361, 45))
    plt.grid(True, linestyle='--', alpha=0.3)
    plt.tight_layout()
    plt.show()

## Execution #######################################################
if __name__ == '__main__':
    main()
