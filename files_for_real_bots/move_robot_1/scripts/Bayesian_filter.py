#!/usr/bin/env python

from filterpy.kalman import KalmanFilter

class BayesianFilter(KalmanFilter):
    def __init__(self, dim_x, dim_z):
        super().__init__(dim_x=dim_x, dim_z=dim_z)
    
    def predict(self, dt=None):
        if dt is not None:
            # Update the state transition matrix and noise covariance matrix
            self.F = np.array([[1, dt], [0, 1]])
            self.Q = self.Q * dt

        super().predict()
