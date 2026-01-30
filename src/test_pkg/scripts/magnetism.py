#!/usr/bin/env python3

## Header #########################################################################################
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from pathlib import Path

## System's modeling for magnetic actuation #######################################################
class MagneticModel:
    _coeffs_cache = None

    def __init__(self, coeff_dir=None):

        if coeff_dir is None:
            coeff_dir = Path(__file__).resolve().parent

        if MagneticModel._coeffs_cache is None:
            MagneticModel._coeffs_cache = {
                m: np.loadtxt(coeff_dir / f"C{m+1}.csv", delimiter=",") for m in range(8)
            }

        self.coeffs = MagneticModel._coeffs_cache

    @staticmethod
    def generate_Poly(position: np.ndarray) -> np.ndarray:
        x, y, z = position.flatten()                          # position must be numpy (3, 1) array

        return np.array([[
            x**5, y**5, z**5,                                                             # Quintic
            x**4 * y, x**4 * z, y**4 * x,
            y**4 * z, z**4 * x, z**4 * y,
            x**3 * y**2, x**3 * z**2, y**3 * x**2,
            y**3 * z**2, z**3 * x**2, z**3 * y**2,
            x**3 * y * z, y**3 * x * z, z**3 * x * y,
            x**2 * y**2 * z, x**2 * z**2 * y, y**2 * z**2 * x,

            x**4, y**4, z**4,                                                             # Quartic
            x**3 * y, x**3 * z, y**3 * x,
            y**3 * z, z**3 * x, z**3 * y,
            x**2 * y**2, x**2 * z**2, y**2 * z**2,
            x**2 * y * z, y**2 * x * z, z**2 * x * y,

            x**3, y**3, z**3,                                                               # Cubic
            x**2 * y, x**2 * z, y**2 * x,
            y**2 * z, z**2 * x, z**2 * y,
            x * y * z,

            x**2, y**2, z**2,                                                           # Quadratic
            x * y, x * z, y * z,

            x, y, z,                                                                       # Linear

            1,                                                                           # Constant
            ]])

    def map_FieldGain(self, position: np.ndarray) -> np.ndarray:
        POLY = self.generate_Poly(position)
        cols = []
        for m in range(8):
            cols.append((POLY.dot(self.coeffs[m])).T)
        return np.hstack(cols)

    def map_GradGain(self, position: np.ndarray, axis: int, h=1e-8) -> np.ndarray:
        cols = []
        for m in range(8):
            POS_FWD = np.array(position, dtype=float)
            POS_BWD = np.array(position, dtype=float)
            POS_FWD[axis] += h
            POS_BWD[axis] -= h
            FWD = self.map_FieldGain(POS_FWD)[:, m:m+1]
            BWD = self.map_FieldGain(POS_BWD)[:, m:m+1]
            cols.append((FWD - BWD) / (2 * h))
        return np.hstack(cols)

## Mapping the rank of the stacked field and gradient gain matrix over XY plane ###################
def map_Rank2D(model: MagneticModel, z: float, tol: float):
    x_rng = range(-70, 71)
    y_rng = range(-70, 71)

    RK_GRID = np.zeros((len(x_rng), len(y_rng)))

    for xi, x in enumerate(x_rng):
        for yj, y in enumerate(y_rng):
            POS = np.array([[x], [y], [z]])

            B_GAIN  = model.map_FieldGain(POS)
            GX_GAIN = model.map_GradGain(POS, axis=0)
            GY_GAIN = model.map_GradGain(POS, axis=1)
            GZ_GAIN = model.map_GradGain(POS, axis=2)

            BG_GAIN = np.vstack((B_GAIN, GZ_GAIN, GY_GAIN[0:2, :]))
            RK_GRID[xi, yj] = np.linalg.matrix_rank(BG_GAIN, tol=tol)

    # rmin = int(Rank_grid.min())
    # rmax = int(Rank_grid.max())
    # ticks = np.arange(rmin, rmax + 1, 1)
    
    rmin, rmax = 7, 8

    bnds = [6.5, 7.5, 8.5]
    norm = BoundaryNorm(bnds, ncolors=plt.cm.viridis.N)

    plt.figure(figsize=(10, 8))

    im = plt.imshow(
        RK_GRID.T,
        origin='lower',
        extent=[min(x_rng), max(x_rng),
                min(y_rng), max(y_rng)],
        interpolation='nearest',
        cmap='viridis',
        norm=norm
    )

    # cbar = plt.colorbar(im, ticks=ticks)
    cbar = plt.colorbar(im, ticks=[7, 8])
    cbar.set_label('Rank of stacked matrix (8x8)')

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Rank map over (x, y) at z = 40")
    plt.tight_layout()
    plt.show()

## Main function ##################################################################################
def main():
    model = MagneticModel()
    map_Rank2D(model, z=40.0, tol=1e-3)

## Execution ######################################################################################
if __name__ == '__main__':
    main()
