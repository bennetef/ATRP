import numpy as np
from dataclasses import dataclass

@dataclass
class KalmanFilterParams:
    process_noise: float
    measurement_noise: float
    initial_error: float

class KalmanFilterGPS:
    def __init__(self, params: KalmanFilterParams):
        """Initialize Kalman Filter for 2D GPS tracking"""
        self.params = params

        self.x = np.array([[0], [0]])  # State: [latitude, longitude]
        self.P = np.eye(2) * self.params.initial_error  # Covariance matrix

        self.Q = np.eye(2) * self.params.process_noise  # Process noise
        self.R = np.eye(2) * self.params.measurement_noise  # Measurement noise

        self.F = np.eye(2)  # State transition matrix
        self.H = np.eye(2)  # Measurement function

    def update(self, z):
        """Update the Kalman Filter with a new GPS measurement (lat, lon)"""
        z = np.array([[z[0]], [z[1]]])  # Convert to column vector

        # Prediction step
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q

        # Measurement update
        y = z - self.H @ x_pred  # Measurement residual
        S = self.H @ P_pred @ self.H.T + self.R  # Residual covariance
        K = P_pred @ self.H.T @ np.linalg.inv(S)  # Kalman Gain

        # Update state
        self.x = x_pred + K @ y
        self.P = (np.eye(2) - K @ self.H) @ P_pred

        return self.x.flatten()  # Return filtered [lat, lon]
