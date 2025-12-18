"""
1D Kalman Filter implementation for state estimation.
"""
import numpy as np


class KalmanFilter1D:
    """
    A simple 1D Kalman Filter for position and velocity estimation.
    
    State vector: [position, velocity]
    """
    
    def __init__(self, dt=0.1, process_noise=0.01, measurement_noise=0.5):
        """
        Initialize the Kalman Filter.
        
        Args:
            dt: Time step (seconds)
            process_noise: Process noise covariance (Q)
            measurement_noise: Measurement noise covariance (R)
        """
        self.dt = dt
        
        # State vector: [position, velocity]
        self.x = np.array([[0.0], [0.0]])
        
        # State covariance matrix
        self.P = np.eye(2) * 1.0
        
        # State transition matrix (Constant Velocity Model)
        self.F = np.array([[1.0, self.dt], 
                          [0.0, 1.0]])
        
        # Measurement matrix (we only measure position)
        self.H = np.array([[1.0, 0.0]])
        
        # Process noise covariance
        self.Q = np.eye(2) * process_noise
        
        # Measurement noise covariance
        self.R = np.array([[measurement_noise]])
    
    def predict(self):
        """
        Prediction step: Project the state forward in time.
        
        Returns:
            Predicted state vector
        """
        # Predicted state estimate
        self.x = self.F @ self.x
        
        # Predicted covariance estimate
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x.copy()
    
    def update(self, measurement):
        """
        Update step: Incorporate new measurement.
        
        Args:
            measurement: Observed position value
            
        Returns:
            Updated state vector
        """
        # Innovation (measurement residual)
        y = measurement - (self.H @ self.x)
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Updated state estimate
        self.x = self.x + K @ y
        
        # Updated covariance estimate
        self.P = (np.eye(2) - K @ self.H) @ self.P
        
        return self.x.copy()
    
    def get_state(self):
        """Get the current state estimate."""
        return self.x.copy()
    
    def get_position(self):
        """Get the estimated position."""
        return float(self.x[0])
    
    def get_velocity(self):
        """Get the estimated velocity."""
        return float(self.x[1])
    
    def reset(self, initial_state=None):
        """
        Reset the filter to initial conditions.
        
        Args:
            initial_state: Optional initial state [position, velocity]
        """
        if initial_state is not None:
            self.x = np.array(initial_state).reshape(2, 1)
        else:
            self.x = np.array([[0.0], [0.0]])
        
        self.P = np.eye(2) * 1.0
