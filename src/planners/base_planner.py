from abc import ABC, abstractmethod
from typing import List, Tuple
from src.simulator.state import Odometry, Twist

class BasePlanner(ABC):
    """
    Abstract base class defining the standard interface for all decision-making algorithms.
    
    """
    def __init__(self, robot_id: int):
        # We store the ID so the planner knows which robot it is controlling
        self.robot_id = robot_id

    @abstractmethod
    def compute_velocity(self, 
                         current_state: Odometry, 
                         target_x: float, 
                         target_y: float, 
                         dynamic_neighbors: List[Odometry], 
                         static_points: List[Tuple[float, float]]) -> Twist:
        """
        Calculates the next velocity command based on state, goal, and sensor data.
        """
        pass