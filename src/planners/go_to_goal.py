import math
from src.planners.base_planner import BasePlanner
from src.simulator.state import Odometry

class GoToGoalPlanner(BasePlanner):
    def __init__(self, robot_id: int, max_speed: float = 0.5):
        super().__init__(robot_id)
        self.max_speed = max_speed

    def compute_preferred_velocity(self, current_state: Odometry, target_x: float, target_y: float) -> tuple:
        """Returns the ideal (vx, vy) vector pointing to the target."""
        dx = target_x - current_state.x
        dy = target_y - current_state.y
        dist = math.hypot(dx, dy)
        
        if dist > 0.05:
            # Normalize and scale to max speed
            pref_vx = (dx / dist) * self.max_speed
            pref_vy = (dy / dist) * self.max_speed
        else:
            pref_vx, pref_vy = 0.0, 0.0
            
        return pref_vx, pref_vy