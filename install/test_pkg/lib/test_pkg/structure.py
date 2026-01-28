#!/usr/bin/env python3

## Header ##########################################################
import numpy as np

## Structure 1 #####################################################
class Config1:                         # Diller et al. maximizing e5

    def __init__(
            self,
            distance,                # magnet-to-magnet distance, mm
            moment_value,      # magnitude of magnetic moment, A*m^2
            number_magnets,                      # number of magnets
            ):                 
        
        self.distance = distance
        self.moment_value = moment_value
        self.number_magnets = number_magnets

        d = self.distance
        m = self.moment_value

        self.rL = {}   # position vectors in local coordinate system
        self.mL = {}      # magnetization in local coordinate system

        self.rL['O'] = np.zeros((3, 1))
        self.mL['O'] = np.array([[0.0], [0.0], [+m]])
            
        if self.number_magnets >= 3:
            self.rL.update({
                '1': np.array([[+d], [0.0], [0.0]]),
                '3': np.array([[-d], [0.0], [0.0]]),
                })
            self.mL.update({
                '1': np.array([[+m], [0.0], [0.0]]),
                '3': np.array([[-m], [0.0], [0.0]]),
                })
        
        if self.number_magnets >= 5:
            self.rL.update({
                '2': np.array([[0.0], [+d], [0.0]]),
                '4': np.array([[0.0], [-d], [0.0]]),
                })
            self.mL.update({
                '2': np.array([[0.0], [-m], [0.0]]),
                '4': np.array([[0.0], [+m], [0.0]]),
                })
        
## Structure 2 #####################################################
class Config2(Config1):                # Diller et al. maximizing e4

    def __init__(
            self, 
            distance, 
            moment_value, 
            number_magnets,
            ):
        
        super().__init__(
            distance, 
            moment_value, 
            number_magnets,
            )

        m = self.moment_value

        if self.number_magnets >= 3:
            self.mL.update({
                '1': np.array([[0.0], [+m], [0.0]]),
                '3': np.array([[0.0], [-m], [0.0]]),
                })

        if self.number_magnets >= 5:
            self.mL.update({
                '2': np.array([[+m], [0.0], [0.0]]),
                '4': np.array([[-m], [0.0], [0.0]]),
                })