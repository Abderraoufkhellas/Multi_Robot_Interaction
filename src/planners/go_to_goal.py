import math
from typing import List, Tuple
from .base_planner import BasePlanner
from src.simulator.state import Odometry, Twist

class GoToGoalPlanner(BasePlanner):
    """
    A simple P-controller that drives directly to a target coordinate.
    Does not perform collision avoidance.
    """
    def __init__(self, robot_id: int, max_linear: float = 0.5, max_angular: float = 1.0):
        super().__init__(robot_id)
        self.max_linear = max_linear
        self.max_angular = max_angular
        
        # Proportional control gains
        self.kp_linear = 0.5
        self.kp_angular = 2.0
        self.goal_tolerance = 0.05

    def compute_velocity(self, 
                         current_state: Odometry, 
                         target_x: float, 
                         target_y: float, 
                         dynamic_neighbors: List[Odometry], 
                         static_points: List[Tuple[float, float]]) -> Twist:
        
        # 1. Calculate distance to goal
        dx = target_x - current_state.x
        dy = target_y - current_state.y
        distance = math.hypot(dx, dy)

        # 2. Stop if we have arrived
        if distance < self.goal_tolerance:
            return Twist(linear_x=0.0, angular_z=0.0)

        # 3. Calculate heading error
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - current_state.theta
        
        # Normalize the error to the range [-pi, pi] so the robot turns the shortest way
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # 4. Proportional Control
        angular_vel = self.kp_angular * heading_error
        linear_vel = self.kp_linear * distance

        # 5. Cap at physical speed limits
        angular_vel = max(min(angular_vel, self.max_angular), -self.max_angular)
        
        
            
        linear_vel = max(min(linear_vel, self.max_linear), -self.max_linear)

        return Twist(linear_x=linear_vel, angular_z=angular_vel)