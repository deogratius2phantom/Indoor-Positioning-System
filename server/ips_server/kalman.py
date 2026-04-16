import numpy as np


class PositionKalmanFilter:
    """Simple 3D Kalman filter with identity state transition.

    `process_noise` controls how much state change is expected between updates.
    `measurement_noise` controls trust in observed trilateration coordinates.
    Increase measurement noise for jittery RSSI environments.
    """

    def __init__(self, process_noise: float = 0.05, measurement_noise: float = 1.0):
        self.state = np.zeros(3)
        self.covariance = np.eye(3)
        self.process_noise = process_noise * np.eye(3)
        self.measurement_noise = measurement_noise * np.eye(3)
        self.initialized = False

    def update(self, measurement: np.ndarray) -> np.ndarray:
        if not self.initialized:
            self.state = measurement.astype(float)
            self.initialized = True
            return self.state

        predicted_state = self.state
        predicted_covariance = self.covariance + self.process_noise

        innovation = measurement - predicted_state
        innovation_covariance = predicted_covariance + self.measurement_noise
        kalman_gain = np.linalg.solve(innovation_covariance.T, predicted_covariance.T).T

        self.state = predicted_state + kalman_gain @ innovation
        self.covariance = (np.eye(3) - kalman_gain) @ predicted_covariance
        return self.state
