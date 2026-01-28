#!/usr/bin/env python3

## Header ##########################################################
from magnetism import *

## Skew-symmetric matrix ###########################################
def genSkewMat(vector):
    x, y, z = vector.flatten()    # value must be numpy (3, 1) array

    return np.array([
        [0.0, -z, +y],
        [+z, 0.0, -x],
        [-y, +x, 0.0],
        ])
## Tikhonov regularization with solution norm constraint ###########
def comTikhonov(
        coefficient_matrix,
        desired_vector,
        parameters,
        norm_maximum, 
        ):
    
    cMat = coefficient_matrix
    D = desired_vector
    
    U, mW, VT = np.linalg.svd(cMat, full_matrices=False)
    residual_norms, solutions = [], []

    for param in parameters:
        filt = mW / (mW**2 + param**2)

        sol = VT.T.dot(filt[:, np.newaxis] * (U.T.dot(D)))
        # sol = (VT.T * filt).dot((U.T.dot(des_vec)))
        res_norm = np.linalg.norm((cMat.dot(sol)) - D)
        sol_norm = np.linalg.norm(sol)

        if sol_norm <= norm_maximum:
            solutions.append(sol)
            residual_norms.append(res_norm)

    best_idx = np.argmin(residual_norms)
    sol_opt = solutions[best_idx]

    return sol_opt

## Solving actuation matrix ########################################
class Evaluation:

    def __init__(
        self,
        structure,                                       # structure
        position,             # estimated position of magnetic robot's center
        orientation,           # estimated orientation of magnetic robot
        # desired_Bz,
        desired_torque,                             # desired torque
        desired_force,                               # desired force
        current_vector=None,
        ):

        self.structure = structure
        self.position = position
        self.orientation = orientation
        # self.desired_Bz = desired_Bz
        self.desired_torque = desired_torque
        self.desired_force = desired_force
        
        pos = position
        theta = orientation

        self.rotZ = np.array([
            [+np.cos(theta), -np.sin(theta), 0.0],
            [+np.sin(theta), +np.cos(theta), 0.0],
            [0.0, 0.0, 1.0],
            ])
        
        self.rW = {} # position vectors in global coordinate system
        self.mW = {}    # magnetization in global coordinate system
        self.sk_R = {}
        self.sk_M = {}
        self.Tm_gain = {}
        self.F_gain = {}
                
        for key, value in structure.rL.items():
            R_rotated = self.rotZ.dot(value)
            R_shifted = pos + self.rotZ.dot(value)
            self.rW[key] = R_shifted
            self.sk_R[key] = genSkewMat(R_rotated)

        for key, value in structure.mL.items():
            M_rotated = self.rotZ.dot(value)
            self.mW[key] = M_rotated
            self.sk_M[key] = genSkewMat(M_rotated)

        for key in structure.rL.keys():
            self.Tm_gain[key] = self.sk_M[key].dot(mapFieldGain(self.rW[key]))
            self.F_gain[key] = np.vstack((
                (self.mW[key].T).dot(mapGradXGain(self.rW[key])),
                (self.mW[key].T).dot(mapGradYGain(self.rW[key])),
                (self.mW[key].T).dot(mapGradZGain(self.rW[key])),
            ))

        # self.Bz_gain = mapFieldGain(pos)[2:3, :]

        # self.Output = np.vstack((np.array([[desired_Bz]]), desired_torque, desired_force))
        self.Output = np.vstack((self.rotZ.dot(desired_torque), self.rotZ.dot(desired_force)))

        self.TmTot_gain = sum(self.Tm_gain.values())
        self.TfTot_gain = sum(self.sk_R[name].dot(self.F_gain[name]) for name in self.F_gain)
        # self.Act_mat = np.vstack((self.Bz_gain, self.TmTot_gain + self.TfTot_gain, self.F_gain['O']))
        self.Act_mat = np.vstack((self.TmTot_gain + self.TfTot_gain, self.F_gain['O']))

        # pseudo-inversed current vector
        # self.Iopt = np.linalg.pinv(self.Act_mat).dot(self.Output)
        
        # Tikhonov regularized current vector
        self.Iopt = comTikhonov(
            coefficient_matrix=self.Act_mat,
            desired_vector=self.Output,
            parameters=np.logspace(-10, 4, 1000),
            norm_maximum=13,
            )

        self.compute_response(current_vector)

    def compute_response(self, current_vector=None):
        if current_vector is None:
            current_vector = self.Iopt
        self.current_vector = current_vector

        self.B = {}
        self.Tm = {}
        self.F = {}

        for key in self.structure.rL.keys():
            self.B[key] = mapFieldGain(self.rW[key]).dot(self.current_vector)

        for key in self.structure.rL.keys():
            self.Tm[key] = self.sk_M[key].dot(self.B[key])
            self.F[key] = np.vstack((
                (self.mW[key].T).dot(mapGradXGain(self.rW[key]).dot(self.current_vector)),
                (self.mW[key].T).dot(mapGradYGain(self.rW[key]).dot(self.current_vector)),
                (self.mW[key].T).dot(mapGradZGain(self.rW[key]).dot(self.current_vector)),
            ))

        self.Ftot = sum(self.F.values())
        self.TmTot = sum(self.Tm.values())
        self.TfTot = sum(self.sk_R[name].dot(self.F[name]) for name in self.F)
        self.Ttot = (self.rotZ.T).dot(self.TmTot + self.TfTot)

    def comField(self, position):
        return mapFieldGain(position).dot(self.current_vector)
