"""
Trajectory generators for drone motion patterns.
"""
import numpy as np


class TrajectoryGenerator:
    """
    Generates reference trajectories for the drone.
    """
    
    def __init__(self, trajectory_type='sine'):
        """
        Initialize trajectory generator.
        
        Args:
            trajectory_type: Type of trajectory ('sine', 'circle', 'square', 'static')
        """
        self.trajectory_type = trajectory_type
        self.start_time = None
    
    def get_position(self, elapsed_time, trajectory_type=None):
        """
        Get the reference position at a given time.
        
        Args:
            elapsed_time: Time since start (seconds)
            trajectory_type: Override the default trajectory type
            
        Returns:
            Tuple (x, y, z) position
        """
        traj_type = trajectory_type or self.trajectory_type
        
        if traj_type == 'sine':
            return self._sine_trajectory(elapsed_time)
        elif traj_type == 'circle':
            return self._circle_trajectory(elapsed_time)
        elif traj_type == 'square':
            return self._square_trajectory(elapsed_time)
        elif traj_type == 'static':
            return self._static_trajectory(elapsed_time)
        else:
            return self._static_trajectory(elapsed_time)
    
    def _sine_trajectory(self, t):
        """Vertical sine wave motion."""
        x = 0.0
        y = 0.0
        z = 2.0 + np.sin(4.0 * t)  # Faster oscillation
        return (x, y, z)
    
    def _circle_trajectory(self, t):
        """Circular motion in the XY plane."""
        radius = 2.0
        height = 2.0
        angular_velocity = 0.5  # rad/s
        
        x = radius * np.cos(angular_velocity * t)
        y = radius * np.sin(angular_velocity * t)
        z = height
        return (x, y, z)
    
    def _square_trajectory(self, t):
        """Square wave vertical motion."""
        period = 4.0
        phase = (t % period) / period
        
        x = 0.0
        y = 0.0
        z = 2.0 if phase < 0.5 else 3.0
        return (x, y, z)
    
    def _static_trajectory(self, t):
        """Static hovering position."""
        return (0.0, 0.0, 2.0)


class NoiseGenerator:
    """
    Generates realistic sensor noise.
    """
    
    def __init__(self, noise_std=0.5, noise_type='gaussian'):
        """
        Initialize noise generator.
        
        Args:
            noise_std: Standard deviation of noise
            noise_type: Type of noise ('gaussian', 'uniform')
        """
        self.noise_std = noise_std
        self.noise_type = noise_type
    
    def add_noise(self, true_value):
        """
        Add noise to a measurement.
        
        Args:
            true_value: Ground truth value
            
        Returns:
            Noisy measurement
        """
        if self.noise_type == 'gaussian':
            noise = np.random.normal(0, self.noise_std)
        elif self.noise_type == 'uniform':
            noise = np.random.uniform(-self.noise_std, self.noise_std)
        else:
            noise = 0.0
        
        return true_value + noise