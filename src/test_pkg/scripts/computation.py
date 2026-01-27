#!/usr/bin/env python3

## Header ##########################################################
from evaluation import *

## Computation #####################################################
class Computation(Evaluation):
    
    def __init__(
            self,
            structure,                                   # structure
            position,         # estimated position of magnetic robot
            orientation,       # estimated orientation of magnetic robot
            # desired_Bz,
            desired_torque,                         # desired torque
            desired_force,                           # desired force
            current_vector,
            ):

        super().__init__(
            structure, 
            position,
            orientation,
            # desired_Bz, 
            desired_torque, 
            desired_force,
            )
        
        self.Iopt = current_vector
        self.B = {}
        self.Tm = {}
        self.F = {}

        for key in structure.rL.keys():
            self.B[key] = mapFieldGain(self.rW[key]).dot(self.Iopt)

        for key in structure.rL.keys():
            self.Tm[key] = self.sk_M[key].dot(self.B[key])
            self.F[key] = np.vstack((
                (self.mW[key].T).dot(mapGradXGain(self.rW[key]).dot(self.Iopt)),
                (self.mW[key].T).dot(mapGradYGain(self.rW[key]).dot(self.Iopt)),
                (self.mW[key].T).dot(mapGradZGain(self.rW[key]).dot(self.Iopt)),
            ))
        
        self.Ftot = sum(self.F.values())
        self.TmTot = sum(self.Tm.values())
        self.TfTot = sum(self.sk_R[name].dot(self.F[name]) for name in self.F)
        self.Ttot = (self.rotZ.T).dot(self.TmTot + self.TfTot)

    def comField(self, position):
        return mapFieldGain(position).dot(self.Iopt)
