import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class PositionVelocityKalmanFilter:
    """
    Uses Filterpy's KalmanFilter class to predict the position and velocity of the drone.
    Args:
        R_pos_diag: measurement noise covariance
        sigma_accel: per-axis standard deviation of the acceleration ignored by the drone model
        P_pos_init: initial position state covariance (how much initial position is trusted)
        P_vel_init: initial velocity state covariance (how much initial velocity is trusted)
    """
    def __init__(self, R_pos_diag, sigma_accel, P_pos_init, P_vel_init):
        self.sigma_accel = np.asarray(sigma_accel, float)
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.R = np.diag(R_pos_diag)
        self.kf.P = np.diag([P_pos_init] * 3 + [P_vel_init] * 3)
        self.initialised = False
        self.kf.H = np.zeros((3, 6))
        for i in range(3):
            self.kf.H[i, i] = 1.0 # i.e. only positions are known


    def initialise(self, pos):
        self.kf.x = np.array([pos[0], pos[1], pos[2], 0.0, 0.0, 0.0])
        self.initialised = True


    def predict(self, dt):
        Q = np.zeros((6, 6))
        F = np.eye(6)
        for i in range(3):
            F[i, i+3] = dt # dx = vx * dt
            accel_uncertainty = Q_discrete_white_noise(dim=2, dt=dt, var=self.sigma_accel[i]**2)
            Q[i, i] = accel_uncertainty[0, 0]
            Q[i, i+3] = accel_uncertainty[0, 1]
            Q[i+3, i] = accel_uncertainty[1, 0]
            Q[i+3, i+3] = accel_uncertainty[1, 1]
        self.kf.F = F
        self.kf.Q = Q
        self.kf.predict()


    def update(self, pos, nis_threshold=None):
        pos_measured = np.asarray(pos, float)
        # self.kf.x is where model thinks we are and we extract the position by multiplying by H.
        innovation = pos_measured - self.kf.H @ self.kf.x
        innovation_covariance = self.kf.H @ self.kf.P @ self.kf.H.T + self.kf.R
        nis = float(innovation @ np.linalg.solve(innovation_covariance, innovation))

        if nis_threshold is not None and nis > nis_threshold:
            return nis, False
        self.kf.update(pos_measured)
        return nis, True
    

    def get_state(self):
        return self.kf.x.copy() 