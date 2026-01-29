#!/usr/bin/env python3

## Header #########################################################################################
import numpy as np
from magnetism import MagneticModel
from structure import RobotDesign

model = MagneticModel()

## Generating skew-symmetric matrix ###############################################################
def generate_SkewMat(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector.flatten()                                  # vector must be numpy (3, 1) array

    return np.array([
        [0.0, -z, +y],
        [+z, 0.0, -x],
        [-y, +x, 0.0],
        ])

## Tikhonov regularization with solution norm constraint ##########################################
def compute_Tikhonov(
        coefficient_matrix: np.ndarray,
        desired_vector: np.ndarray,
        parameter_range: np.ndarray,
        norm_maximum: float,
        ) -> np.ndarray:
    
    COEFF = coefficient_matrix
    DES_VEC = desired_vector
    params = parameter_range
    
    U, W, VT = np.linalg.svd(COEFF, full_matrices=False)
    resi_norms, sols = [], []

    for param in params:
        FILT = W / (W**2 + param**2)

        SOL = VT.T.dot(FILT[:, np.newaxis] * (U.T.dot(DES_VEC)))
        # SOL = (VT.T * FILT).dot((U.T.dot(DES_VEC)))
        resi_norm = np.linalg.norm((COEFF.dot(SOL)) - DES_VEC)
        sol_norm = np.linalg.norm(SOL)

        if sol_norm <= norm_maximum:
            sols.append(SOL)
            resi_norms.append(resi_norm)

    best_idx = np.argmin(resi_norms)
    SOL_OPT = sols[best_idx]

    return SOL_OPT

## Solving actuation matrix #######################################################################
class ActuationModel:

    def __init__(
        self,
        robot_design: RobotDesign,                                                   # robot_design
        robot_position: np.ndarray,                    # estimated position of magnetic robot's COM
        robot_orientation: float,                         # estimated orientation of magnetic robot
        desired_torque: np.ndarray,                                                # desired torque
        desired_force: np.ndarray,                                                  # desired force
        solve_method: str = "tikhonov",  # solve method ("tikhonov" or "pinv"), default: "tikhonov"
        ):

        self.robot_design = robot_design
        self.robot_position = robot_position
        self.robot_orientation = robot_orientation
        self.desired_torque = desired_torque
        self.desired_force = desired_force
        self.solve_method = solve_method
        
        POS = self.robot_position
        theta = self.robot_orientation
        T_DES = self.desired_torque
        F_DES = self.desired_force

        self.ROT_Z = np.array([
            [+np.cos(theta), -np.sin(theta), 0.0],
            [+np.sin(theta), +np.cos(theta), 0.0],
            [0.0, 0.0, 1.0],
            ])
        
        self.rw = {}                           # robot_position vectors in global coordinate system
        self.mw = {}                                    # magnetization in global coordinate system
        self.sk_rw = {}
        self.sk_mw = {}
        self.tm_gain = {}
        self.f_gain = {}
                
        for key, value in robot_design.rb.items():
            R_ROT = self.ROT_Z.dot(value)
            R_SHIFT = POS + self.ROT_Z.dot(value)
            self.rw[key] = R_SHIFT
            self.sk_rw[key] = generate_SkewMat(R_ROT)

        for key, value in robot_design.mb.items():
            M_ROT = self.ROT_Z.dot(value)
            self.mw[key] = M_ROT
            self.sk_mw[key] = generate_SkewMat(M_ROT)

        for key in robot_design.rb.keys():
            self.tm_gain[key] = self.sk_mw[key].dot(model.map_FieldGain(self.rw[key]))
            self.f_gain[key] = np.vstack((
                (self.mw[key].T).dot(model.map_GradGain(self.rw[key], axis=0)),
                (self.mw[key].T).dot(model.map_GradGain(self.rw[key], axis=1)),
                (self.mw[key].T).dot(model.map_GradGain(self.rw[key], axis=2)),
            ))

        self.OUTPUT = np.vstack((self.ROT_Z.dot(T_DES), self.ROT_Z.dot(F_DES)))

        self.TM_GAIN = sum(self.tm_gain.values())
        self.TF_GAIN = sum(self.sk_rw[name].dot(self.f_gain[name]) for name in self.f_gain)
        self.ACT_MAT = np.vstack((self.TM_GAIN + self.TF_GAIN, self.f_gain['O']))
    
        if self.solve_method == "pinv":
            self.CURR_VEC = np.linalg.pinv(self.ACT_MAT).dot(self.OUTPUT)

        elif self.solve_method == "tikhonov":
            self.CURR_VEC = compute_Tikhonov(
                coefficient_matrix=self.ACT_MAT,
                desired_vector=self.OUTPUT,
                parameter_range=np.logspace(-10, 4, 1000),
                norm_maximum=13.0,
            )

        else:
            raise ValueError(f"Unknown method: {self.solve_method}")
        
    def compute_Response(self):

        self.b = {}
        self.tm = {}
        self.f = {}

        for key in self.robot_design.rb.keys():
            self.b[key] = model.map_FieldGain(self.rw[key]).dot(self.CURR_VEC)

        for key in self.robot_design.rb.keys():
            self.tm[key] = self.sk_mw[key].dot(self.b[key])
            self.f[key] = np.vstack((
                (self.mw[key].T).dot(model.map_GradGain(self.rw[key], axis=0).dot(self.CURR_VEC)),
                (self.mw[key].T).dot(model.map_GradGain(self.rw[key], axis=1).dot(self.CURR_VEC)),
                (self.mw[key].T).dot(model.map_GradGain(self.rw[key], axis=2).dot(self.CURR_VEC)),
            ))

        self.F = sum(self.f.values())
        self.TM = sum(self.tm.values())
        self.TF = sum(self.sk_rw[name].dot(self.f[name]) for name in self.f)
        self.T = (self.ROT_Z.T).dot(self.TM + self.TF)