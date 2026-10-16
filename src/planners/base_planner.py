from abc import ABC, abstractmethod
from src.simulator.state import Twist, Odometry

class BasePlanner(ABC):
    """
    Abstract base class defining the standard interface for all decision-making algorithms.
    
    """
    def __init__(self, robot_id: int):
        # We store the ID so the planner knows which robot it is controlling
        self.robot_id = robot_id

    @abstractmethod
    def compute_velocity(self, current_state: Odometry, target_x: float, target_y: float) -> Twist:
        """
        Calculates the required velocity command to move the robot toward its goal.
        
        
        """
        pass