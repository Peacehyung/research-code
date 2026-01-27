#!/usr/bin/env python3

## Header ##########################################################
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
from pathlib import Path

## Polynomial modeling #############################################
def genPoly(position):
    x, y, z = position.flatten()  # position must be numpy (3, 1) array
    
    return np.array([[
        x**5, y**5, z**5,                                  # Quintic
        x**4 * y, x**4 * z, y**4 * x,
        y**4 * z, z**4 * x, z**4 * y,
        x**3 * y**2, x**3 * z**2, y**3 * x**2, 
        y**3 * z**2, z**3 * x**2, z**3 * y**2,
        x**3 * y * z, y**3 * x * z, z**3 * x * y,
        x**2 * y**2 * z, x**2 * z**2 * y, y**2 * z**2 * x,
                      
        x**4, y**4, z**4,                                  # Quartic
        x**3 * y, x**3 * z, y**3 * x, 
        y**3 * z, z**3 * x, z**3 * y,
        x**2 * y**2, x**2 * z**2, y**2 * z**2,
        x**2 * y * z, y**2 * x * z, z**2 * x * y,
                      
        x**3, y**3, z**3,                                    # Cubic
        x**2 * y, x**2 * z, y**2 * x, 
        y**2 * z, z**2 * x, z**2 * y,
        x * y * z,
                      
        x**2, y**2, z**2,                                # Quadratic
        x * y, x * z, y * z,
                      
        x, y, z,                                            # Linear
                      
        1,                                                # Constant
        ]])                                        
    
## Coefficient data ################################################
_coeff_dir = Path(__file__).resolve().parent
C = {}
for m in range(8):
    C[m] = np.loadtxt(_coeff_dir / f"C{m+1}.csv", delimiter=',')

## Mapping current normalized magnetic field and gradient gain #####
def mapFieldGain(position):
    poly = genPoly(position)
    cols = []
    for m in range(8):
        cols.append((poly.dot(C[m])).T)   
    return np.hstack(cols)

def mapGradXGain(position, h=1e-8):
    cols = []
    for m in range(8):
        pos_fwd = np.array(position, dtype=float)
        pos_bwd = np.array(position, dtype=float)
        pos_fwd[0] += h
        pos_bwd[0] -= h
        fwd = mapFieldGain(pos_fwd)[:, m:m+1]
        bwd = mapFieldGain(pos_bwd)[:, m:m+1]
        cols.append((fwd - bwd) / (2*h))
    return np.hstack(cols)                          

def mapGradYGain(position, h=1e-8):
    cols = []
    for m in range(8):
        pos_fwd = np.array(position, dtype=float)
        pos_bwd = np.array(position, dtype=float)
        pos_fwd[1] += h
        pos_bwd[1] -= h
        fwd = mapFieldGain(pos_fwd)[:, m:m+1]
        bwd = mapFieldGain(pos_bwd)[:, m:m+1]
        cols.append((fwd - bwd) / (2*h))
    return np.hstack(cols)                

def mapGradZGain(position, h=1e-8):
    cols = []
    for m in range(8):
        pos_fwd = np.array(position, dtype=float)
        pos_bwd = np.array(position, dtype=float)
        pos_fwd[2] += h
        pos_bwd[2] -= h
        fwd = mapFieldGain(pos_fwd)[:, m:m+1]
        bwd = mapFieldGain(pos_bwd)[:, m:m+1]
        cols.append((fwd - bwd) / (2*h))
    return np.hstack(cols)

## Mapping the rank of the stacked field and gradient gain matrix ##
def mapRank2D():
    X_range = range(-70, 71)
    Y_range = range(-70, 71)

    Rank_grid = np.zeros((len(X_range), len(Y_range)))

    for xi, x in enumerate(X_range):
        for yj, y in enumerate(Y_range):
            pos = np.array([[x], [y], [40.0]])

            B_gain  = mapFieldGain(pos)
            Gx_gain = mapGradXGain(pos)       
            Gy_gain = mapGradYGain(pos)
            Gz_gain = mapGradZGain(pos)       

            BG_mat = np.vstack((B_gain, Gz_gain, Gy_gain[0:2, :]))  
            Rank_grid[xi, yj] = np.linalg.matrix_rank(BG_mat, tol=1e-3)

    # rmin = int(Rank_grid.min())
    # rmax = int(Rank_grid.max())
    # ticks = np.arange(rmin, rmax + 1, 1)
    
    rmin, rmax = 7, 8

    bounds = [6.5, 7.5, 8.5]
    norm = BoundaryNorm(bounds, ncolors=plt.cm.viridis.N)

    plt.figure(figsize=(10, 8))

    im = plt.imshow(
        Rank_grid.T,
        origin='lower',
        extent=[min(X_range), max(X_range),
                min(Y_range), max(Y_range)],
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

def showFullRankRegion3D():
    X_range = range(-50, 51)
    Y_range = range(-50, 51)
    Z_range = range(-50, 51)

    Rank_grid = np.zeros((len(X_range), len(Y_range), len(Z_range)))

    for xi, x in enumerate(X_range):
        for yj, y in enumerate(Y_range):
            for zk, z in enumerate(Z_range):

                pos = np.array([[x], [y], [z]])

                B_gain  = mapFieldGain(pos)
                Gx_gain = mapGradXGain(pos)       
                Gy_gain = mapGradYGain(pos)
                Gz_gain = mapGradZGain(pos)       

                BG_mat = np.vstack((B_gain, Gz_gain, Gy_gain[0:2, :]))  
                Rank_grid[xi, yj, zk] = np.linalg.matrix_rank(BG_mat, tol=1e-4)

    full_rank_mask = (Rank_grid == 8)

    xs, ys, zs = np.where(full_rank_mask)

    xs = np.array(X_range)[xs]
    ys = np.array(Y_range)[ys]
    zs = np.array(Z_range)[zs]

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(xs, ys, zs, s=2, c='blue', alpha=0.3)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title('Full-rank region in 3D space')

    plt.show()

## Execution #######################################################
def main():
    mapRank2D()
    # showFullRankRegion3D()


if __name__ == '__main__':
    main()
