#!/usr/bin/env python3

## Header #####################################################################################
import numpy as np

## Robot design for 6-DOF (Diller et al.) #####################################################
class RobotDesign:

    def __init__(
            self,
            magnet_distance: float,                      # magnet-to-magnet distance (unit: mm)
            moment_value: float,                   # magnitude of magnetic moment (unit: A*m^2)
            number_magnets: int,                                            # number of magnets
            pattern: str = "e5",                         # magnetization pattern, default: "e5"
            ):                 
        
        self.magnet_distance = magnet_distance
        self.moment_value = moment_value
        self.number_magnets = number_magnets
        self.pattern = pattern

        d = self.magnet_distance
        m = self.moment_value

        self.RL = {}                              # position vectors in local coordinate system
        self.ML = {}                                 # magnetization in local coordinate system

        self.RL['O'] = np.zeros((3, 1))
        self.ML['O'] = np.array([[0.0], [0.0], [+m]])
            
        if self.number_magnets >= 3:
            self.RL.update({
                '1': np.array([[+d], [0.0], [0.0]]), 
                '3': np.array([[-d], [0.0], [0.0]]),
                })
        
        if self.number_magnets >= 5:
            self.RL.update({
                '2': np.array([[0.0], [+d], [0.0]]),
                '4': np.array([[0.0], [-d], [0.0]]),
                })

        self._apply_pattern()

    def _apply_pattern(self):
        m = self.moment_value

        if self.pattern == "e5":                          # magnetization pattern maximizing e5
            if self.number_magnets >= 3:
                self.ML.update({
                    '1': np.array([[+m], [0.0], [0.0]]),
                    '3': np.array([[-m], [0.0], [0.0]]),
                    })

            if self.number_magnets >= 5:
                self.ML.update({
                    '2': np.array([[0.0], [-m], [0.0]]),
                    '4': np.array([[0.0], [+m], [0.0]]),
                    })
            return

        if self.pattern == "e4":                          # magnetization pattern maximizing e4
            if self.number_magnets >= 3:
                self.ML.update({
                    '1': np.array([[0.0], [+m], [0.0]]),
                    '3': np.array([[0.0], [-m], [0.0]]),
                    })

            if self.number_magnets >= 5:
                self.ML.update({
                    '2': np.array([[+m], [0.0], [0.0]]),
                    '4': np.array([[-m], [0.0], [0.0]]),
                    })
            return

        raise ValueError(f"Unknown pattern: {self.pattern}")
