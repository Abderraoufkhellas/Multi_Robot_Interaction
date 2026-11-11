import numpy as np
from dataclasses import dataclass
from typing import List

from src.simulator.state import Odometry

@dataclass
class NeighborTrajectory:
    """
    Stores the predicted future coordinates of a dynamic neighbor.
    expected_path: A NumPy array of shape (2, N) where row 0 is X and row 1 is Y.
    """
    expected_path: np.ndarray

class NeighborPredictor:
    def __init__(self, N: int, dt: float):
        self.N = N
        self.dt = dt
        self.time_steps = np.arange(self.N) * self.dt 

    def predict_trajectory(self, neighbors: List[Odometry]) -> List[NeighborTrajectory]:
        """
        Predicts future positions using CTRV (Constant Turn Rate and Velocity).
        If angular velocity is near zero, falls back to standard CV (straight line).
        """
        predictions = []
        
        for neighbor in neighbors:
            x0 = neighbor.x
            y0 = neighbor.y
            theta0 = neighbor.theta
            v = neighbor.v
            
            # Assuming your Odometry class tracks angular velocity (w / omega)
            # If your attribute is named differently (e.g., neighbor.omega), adjust here.
            w = getattr(neighbor, 'w', 0.0) 
            
            # Prevent divide-by-zero for straight-line driving
            if abs(w) < 1e-4:
                # Fallback to Constant Velocity (CV) straight line
                vx = v * np.cos(theta0)
                vy = v * np.sin(theta0)
                
                x_future = x0 + (vx * self.time_steps)
                y_future = y0 + (vy * self.time_steps)
            else:
                # CTRV (Arc/Curve Prediction)
                theta_future = theta0 + (w * self.time_steps)
                
                x_future = x0 + (v / w) * (np.sin(theta_future) - np.sin(theta0))
                y_future = y0 - (v / w) * (np.cos(theta_future) - np.cos(theta0))
            
            path = np.vstack((x_future, y_future))
            predictions.append(NeighborTrajectory(expected_path=path))
            
        return predictions